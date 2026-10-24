#![no_std]
#![no_main]

use defmt_rtt as _;

mod panic_capture;

pub mod pac {
    pub use embassy_stm32::Peripherals;
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

mod ankyra;
mod app_types;
mod board_config;
mod board_init;
mod commissioning_backend;
mod firmware_ctx;
mod heartbeat;
mod helpers;
#[cfg(feature = "hfkp-encoder")]
mod hfkp_encoder;
mod interrupts;
mod state;
#[cfg(feature = "hfkp-encoder")]
mod tmc_bus;
mod tmc_control;
mod tmc_diagnostics;
mod tmc_dispatch;
mod transport;
mod trigger_domain;
mod usb_task;
#[cfg(feature = "trace")]
mod usb_trace;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [UART4, UART5, USART3])]
mod app {
    use core::sync::atomic::Ordering::Relaxed;

    use crate::ankyra::constants::CLOCK_FREQ;
    use embassy_stm32::gpio::Input;
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::spi::Spi;
    use embassy_stm32::wdg::IndependentWatchdog;
    use embassy_stm32::{bind_interrupts, peripherals};
    use embedded_hal_bus::spi::ExclusiveDevice;
    use foci_klipper::state::State;
    use foci_tmc4671::spi::Tmc4671Spi;
    use rtic_monotonics::fugit::{self, ExtU64};
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::Receiver;

    use crate::app_types::{CycleDelay, TmcSpiDevice};
    use crate::helpers::lifecycle::*;
    use crate::helpers::stepper::*;
    use crate::helpers::stepper_diag::*;
    use crate::helpers::trsync::*;
    use crate::state::stepper::*;
    use crate::state::tmc::*;
    use crate::trigger_domain::{
        TRIGGER_DOMAIN, TRIGGER_TIMER_MIN_LEAD_TICKS, TriggerDomain, arm_timer_at, disarm_timer,
        reprogram_timer, set_timer_active, with_trigger_domain,
    };
    use foci_firmware::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlots;
    use foci_firmware::context::stepper::{
        StepperArmCheckBoard, StepperStopCheckBoard, StepperTimerStateBoard,
    };
    use foci_firmware::context::stepper_reconcile::drain_stepper_reconciliation;
    use foci_firmware::diagnostic::StepperEventReason;
    use foci_firmware::state::lifecycle::*;
    use foci_firmware::state::stepper::{NO_STEPPER_CHANNEL, STEPPER_DEADLINE_MISS_CHANNEL};
    use foci_firmware::state::tmc::{TMC_DISABLE_SIGNAL, TMC_QUEUE_FLUSH};
    use foci_firmware::state::trsync::*;
    use foci_firmware::stepper_runtime::StepDeadlineStatus;

    bind_interrupts!(struct Irqs {
        DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
        DMA2_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH3>;
    });

    stm32_tim2_monotonic!(Mono, 1_000_000);

    static USB_PUMP_COUNTERS: foci_firmware::usb_pump::UsbPumpCounters =
        foci_firmware::usb_pump::UsbPumpCounters::new();
    static USB_PUMP_EMITTER: crate::usb_task::Emitter = crate::usb_task::Emitter::new();

    #[cfg(feature = "usb-p1-probe")]
    static P1_PROBE_ITERS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    #[cfg(feature = "usb-p1-probe")]
    static P1_PROBE_MAX_GAP_US: core::sync::atomic::AtomicU32 =
        core::sync::atomic::AtomicU32::new(0);
    #[cfg(feature = "usb-p1-probe")]
    static P1_PROBE_LAST_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

    use foci_firmware::tmc::TmcRequest;

    use crate::board_config::{TMC_CHANNEL_COUNT, TMC_CONFIG};
    use crate::tmc_control::board::{
        openffboard_backend, refresh_velocity_scale, stepper_state, tmc_state,
    };
    use crate::tmc_control::request::{handle_commission_wait_reject, handle_tmc_control_request};
    use foci_firmware::tmc_control::TmcControlRequestTail;

    #[shared]
    struct Shared {
        app_state: State,
    }

    type IntegralCandidatesTaskLocal = IntegralRetentionSlots<{ TMC_CHANNEL_COUNT as usize }>;

    #[local]
    struct Local {
        tmc: Tmc4671Spi<TmcSpiDevice>,
        tmc_enable: Output<'static>,
        tmc_status: Input<'static>,
        tmc_rx: Receiver<'static, TmcRequest, TMC_CHANNEL_SIZE>,
        wdg: IndependentWatchdog<'static, peripherals::IWDG>,
        integral_candidates: IntegralCandidatesTaskLocal,
        adc_internal:
            embassy_stm32::adc::Adc<'static, crate::board_config::InternalTempAdcInstance>,
        temp_channel: embassy_stm32::adc::Temperature,
        mcu_temp_calibration: foci_firmware::mcu_temp::FactoryCalibration,
    }

    #[init(local = [
        tmc_channel: rtic_sync::channel::Channel<TmcRequest, TMC_CHANNEL_SIZE> =
            rtic_sync::channel::Channel::new(),
        uid_str: [u8; 24] = [0; 24],
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        crate::board_init::paint_stack();
        foci_firmware::diagnostic::recover_last_boot_breakaway_breadcrumb();
        defmt::info!("FOCI OpenFFBoard starting");

        if let Some(record) = crate::panic_capture::recover() {
            defmt::error!(
                "recovered last-boot fault: kind={} line={} pc={:#010x}",
                record.kind,
                record.line,
                record.pc
            );
            foci_firmware::diagnostic::set_last_panic(record);
        }

        // SAFETY: init runs exactly once, before any other task starts.
        unsafe {
            crate::state::stepper::init_step_queue();
        }

        let p = embassy_stm32::init(crate::board_init::rcc_config());
        defmt::info!("SYSCLK: 168 MHz, USB: 48 MHz");
        crate::board_init::verify_dma_interrupt_priority();
        crate::board_init::enable_debug_clock_during_sleep();

        {
            let csr = embassy_stm32::pac::RCC.csr().read();
            defmt::info!(
                "reset cause: LPWRRSTF={} WWDGRSTF={} IWDGRSTF={} SFTRSTF={} PORRSTF={} PINRSTF={} BORRSTF={}",
                csr.lpwrrstf(),
                csr.wwdgrstf(),
                csr.wdgrstf(),
                csr.sftrstf(),
                csr.porrstf(),
                csr.padrstf(),
                csr.borrstf(),
            );
            embassy_stm32::pac::RCC.csr().modify(|w| w.set_rmvf(true));
        }

        crate::board_init::init_tim5_clock();
        #[cfg(feature = "trace")]
        {
            crate::usb_trace::note_trace_boot();
        }
        crate::board_init::init_stepdir_gpio();
        defmt::info!("TIM5 enabled for Klipper clock @ 84 MHz, STEP/DIR GPIO configured");
        Mono::start(84_000_000);

        let tmc_enable = Output::new(p.PE7, Level::Low, Speed::Low);

        crate::board_init::init_brake_pin(p.PE10);
        let tmc_status = crate::board_init::init_tmc_status_pin(p.PE8);

        let mut wdg = IndependentWatchdog::new(p.IWDG, 10_000_000);
        wdg.unleash();
        defmt::info!("IWDG started: 10s timeout");

        let mut spi_config = embassy_stm32::spi::Config::default();
        spi_config.mode = embassy_stm32::spi::Mode {
            polarity: embassy_stm32::spi::Polarity::IdleHigh,
            phase: embassy_stm32::spi::Phase::CaptureOnSecondTransition,
        };
        spi_config.frequency = embassy_stm32::time::Hertz(8_000_000);

        let spi1 = Spi::new(
            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH2, Irqs, spi_config,
        );

        let spi_cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
        let spi_dev = ExclusiveDevice::new(spi1, spi_cs, CycleDelay)
            .expect("ExclusiveDevice::new is infallible");
        #[cfg(feature = "hfkp-encoder")]
        let spi_dev = crate::tmc_bus::install(spi_dev);
        let tmc = Tmc4671Spi::new(spi_dev)
            .with_retry_policy(foci_tmc4671::spi::RetryPolicy::production());

        let (tmc_tx, tmc_rx) = ctx.local.tmc_channel.split();
        critical_section::with(|cs| {
            TMC_REQUEST_SENDER.borrow(cs).replace(Some(tmc_tx));
        });

        defmt::info!("SPI1 configured: 8 MHz, Mode 3, DMA");

        crate::board_init::init_usb_pins(p.PA11, p.PA12);
        crate::board_init::enable_usb_clock();

        let uid = crate::board_init::read_stm32_uid(ctx.local.uid_str);
        #[cfg(feature = "trace")]
        crate::usb_trace::set_trace_serial(uid);

        // SAFETY: Called exactly once, before any other klipper-usb function.
        // base_addr is the USB_OTG_FS peripheral base on STM32F407.
        unsafe {
            foci_usb_stm32::usb_init(&foci_usb_stm32::UsbConfig {
                base_addr: 0x5000_0000,
                irq_num: 67,
                trdt: 6,
                vbus_detection: false, // F407 uses NOVBUSSENS, not GOTGCTL BVAL
                double_buffer_tx: true,
                vid: 0x1D50,
                pid: 0x614E,
                manufacturer: "foci",
                product: crate::board_config::USB_PRODUCT,
                serial: uid,
            });
        }

        crate::board_init::enable_dwt_cyccnt();

        // SAFETY: init runs once before any task starts. No concurrent access.
        unsafe {
            core::ptr::addr_of_mut!(TRIGGER_DOMAIN)
                .cast::<TriggerDomain>()
                .write(TriggerDomain::new());
        }
        let adc_internal = embassy_stm32::adc::Adc::new(p.ADC1);
        let temp_channel = adc_internal.enable_temperature();
        let mcu_temp_calibration = crate::board_config::read_mcu_temp_calibration();

        #[cfg(feature = "hfkp-encoder")]
        {
            crate::hfkp_encoder::init(crate::hfkp_encoder::HfkpPins {
                uart4: p.UART4,
                usart3: p.USART3,
                rx_dma: p.DMA1_CH2,
                timer: p.TIM7,
                tx: p.PB10,
                rx: p.PA1,
                de: p.PB4,
                de_timer: p.TIM3,
            });
            defmt::info!("HF-KP encoder feed configured");
        }

        stepper_reconcile::spawn().expect("stepper reconciliation worker must spawn exactly once");
        tmc_control::spawn().expect("tmc_control task must spawn exactly once");
        usb_task::spawn().expect("usb_task must spawn exactly once");
        stats::spawn().expect("stats task must spawn exactly once");
        watchdog::spawn().expect("watchdog task must spawn exactly once");
        analog_sampling::spawn().expect("analog_sampling task must spawn exactly once");
        trigger_drain_task::spawn().expect("trigger drain worker must spawn exactly once");

        defmt::info!("FOCI OpenFFBoard initialized");

        (
            Shared {
                app_state: State::new(),
            },
            Local {
                tmc,
                tmc_enable,
                tmc_status,
                tmc_rx,
                wdg,
                integral_candidates: IntegralCandidatesTaskLocal::new(),
                adc_internal,
                temp_channel,
                mcu_temp_calibration,
            },
        )
    }

    /// Sleep until the next interrupt with interrupts masked, and record the sleep as an idle lap.
    fn measure_masked_sleep() {
        // PRIMASK set so wfi wakes on a pending IRQ without running its handler.
        cortex_m::interrupt::free(|_| {
            let sleep_start = read_tim5();
            rtic::export::wfi();
            let slept = read_tim5().wrapping_sub(sleep_start);
            foci_firmware::cpu_load::record_idle_lap(slept);
        });
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            measure_masked_sleep();
        }
    }

    use crate::ankyra::replies::{emit_phase_reply, emit_terminal_reply};

    #[task(
        priority = 1,
        shared = [app_state],
        local = [
            tmc,
            tmc_enable,
            tmc_status,
            tmc_rx,
            integral_candidates,
            driver: foci_firmware::tmc_control::ChannelDriver =
                foci_firmware::tmc_control::ChannelDriver::new(0),
        ]
    )]
    async fn tmc_control(mut ctx: tmc_control::Context) {
        use embassy_futures::select::{Either, select};
        use foci_firmware::tmc_control::scheduler::{
            ChannelMode, ChannelScheduler, MaintenanceArm, UnitClass,
        };
        use foci_firmware::tmc_control::{AuditResult, CommissionNext, UnitOutcome};

        let tmc = ctx.local.tmc;
        let enable_pin = ctx.local.tmc_enable;
        let status_pin = ctx.local.tmc_status;
        let rx = ctx.local.tmc_rx;
        let integral_candidates = ctx.local.integral_candidates;
        let driver = ctx.local.driver;

        let signals = foci_firmware::tmc_control::TmcControlSignals::new(
            &SHUTDOWN_ACTIVE,
            &TMC_QUEUE_FLUSH,
            &FAULT_ACTIVE,
            &TMC_DISABLE_SIGNAL,
        );

        let tmc_control_config = foci_firmware::tmc_control::TmcControlConfig::new(
            CLOCK_FREQ,
            TMC_CONFIG.pwm_freq_hz(),
            TMC_CONFIG.current_scale_ma_per_lsb,
            TMC_CONFIG,
        );

        let now_us = || Mono::now().duration_since_epoch().to_micros();
        let mut scheduler = ChannelScheduler::<1>::new(now_us(), [1_000]);
        let mut group_all_ready = true;

        let mut shutdown_safe_state_applied = false;

        {
            let mut backend = crate::tmc_control::board::openffboard_backend(tmc, enable_pin);
            if let Err(failure) = foci_firmware::tmc_control::bootstrap_tmc_safety(
                &mut backend,
                0,
                crate::tmc_control::board::STATUS_CRITICAL_MASK,
            )
            .await
            {
                let reason = failure.shutdown_reason(0);
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        foci_firmware::dispatch_shutdown!(
                            reason,
                            read_tim5(),
                            &mut crate::_ankyra_config::Sender
                        );
                    }
                });
            }
        }

        'tmc_loop: loop {
            #[cfg(feature = "usb-p1-probe")]
            {
                let now = Mono::now().ticks() as u32;
                let last = P1_PROBE_LAST_US.swap(now, Relaxed);
                if last != 0 {
                    P1_PROBE_MAX_GAP_US.fetch_max(now.wrapping_sub(last), Relaxed);
                }
                P1_PROBE_ITERS.fetch_add(1, Relaxed);
            }

            let status_pin_asserted = status_pin.is_high();
            tmc_state()
                .status_pin_asserted
                .store(status_pin_asserted, Relaxed);

            if driver.runtime.restore_cancel_pending() {
                let mut backend = openffboard_backend(tmc, enable_pin);
                driver
                    .runtime
                    .drive_restore_cancel(
                        &mut backend,
                        tmc_state(),
                        stepper_state(),
                        foci_firmware::autotune::CommissionError::Shutdown,
                    )
                    .await;
            }

            let emergency_pending =
                TMC_DISABLE_SIGNAL.signaled() || FAULT_ACTIVE.load(Relaxed) || status_pin_asserted;
            if !emergency_pending && tmc_state().cancel_requested.load(Relaxed) {
                let mut backend = openffboard_backend(tmc, enable_pin);
                if let Some(foci_firmware::commission_dispatch::CommissionOutcome::Terminal {
                    reply,
                    actions,
                }) = driver
                    .runtime
                    .drive_operator_cancel(
                        &mut backend,
                        tmc_state(),
                        stepper_state(),
                        TMC_CONFIG.pwm_maxcnt,
                    )
                    .await
                {
                    #[cfg(feature = "trace")]
                    {
                        driver.commission_trace_phase = None;
                    }
                    {
                        let mut state = driver.runtime.dispatch_state(tmc_state(), stepper_state());
                        apply_stepper_post_action(actions.stepper, &mut state);
                    }
                    foci_firmware::diagnostic::mark_p1_reply_construction_start();
                    emit_terminal_reply(0, reply);
                    foci_firmware::diagnostic::mark_p1_reply_construction_end();
                }
            }

            if SHUTDOWN_ACTIVE.load(Relaxed) {
                if !shutdown_safe_state_applied {
                    #[cfg(feature = "trace")]
                    {
                        driver.commission_trace_phase = None;
                    }
                    {
                        let mut backend = openffboard_backend(tmc, enable_pin);
                        foci_firmware::tmc_control::lifecycle::apply_shutdown_safe_state(
                            &mut driver.runtime,
                            &mut backend,
                            tmc_state(),
                            stepper_state(),
                        )
                        .await;
                    }
                    tmc_state().commission_in_flight.store(false, Relaxed);
                    stepper_stop_from_p1(0);
                    shutdown_safe_state_applied = true;
                    integral_candidates.invalidate_all();
                }
                Mono::delay(1.millis()).await;
                continue;
            }
            shutdown_safe_state_applied = false;

            let lease_check_now = read_tim5();
            match crate::tmc_control::effects::check_lease(
                SHUTDOWN_ACTIVE.load(Relaxed),
                lease_check_now,
            ) {
                crate::tmc_control::effects::LeaseCheckDisposition::EnterClockSyncShutdown => {
                    let snapshot = crate::tmc_control::effects::lease_snapshot(lease_check_now);
                    defmt::warn!(
                        "clock-sync lease expired: now={} last_liveness={} elapsed={} has_received_valid_clock={}",
                        lease_check_now,
                        snapshot.last_liveness_ticks,
                        snapshot.elapsed_ticks,
                        snapshot.has_received_valid_clock,
                    );
                    ctx.shared.app_state.lock(|state| {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            foci_firmware::dispatch_shutdown!(
                                foci_firmware::shutdown::ShutdownReason::HostClockSyncExpired,
                                read_tim5(),
                                &mut crate::_ankyra_config::Sender
                            );
                        }
                    });
                    continue;
                }
                crate::tmc_control::effects::LeaseCheckDisposition::Proceed => {}
            }

            if TMC_QUEUE_FLUSH.load(Relaxed) {
                integral_candidates.invalidate_all();
                while rx.try_recv().is_ok() {}
                let commission_was_active = driver.runtime.is_commission_active();
                crate::tmc_control::effects::apply_actions(
                    driver
                        .runtime
                        .poll_queue_flush(&signals, tmc_state(), stepper_state()),
                    |reason| {
                        ctx.shared.app_state.lock(|state| {
                            if crate::firmware_ctx::enter_shutdown(state) {
                                crate::tmc_control::effects::emit_shutdown(reason);
                            }
                        });
                    },
                );
                if commission_was_active {
                    #[cfg(feature = "trace")]
                    {
                        driver.commission_trace_phase = None;
                    }
                    let mut backend = openffboard_backend(tmc, enable_pin);
                    let mut state = driver.runtime.dispatch_state(tmc_state(), stepper_state());
                    foci_firmware::tmc_control::lifecycle::disarm_motor(&mut backend, &mut state)
                        .await;
                }
                tmc_state().commission_in_flight.store(false, Relaxed);
                defmt::info!("TMC request queue flushed after config_reset");
            }

            if TMC_DISABLE_SIGNAL.try_take().is_some()
                || FAULT_ACTIVE.load(Relaxed)
                || status_pin_asserted
            {
                let deadline_channel =
                    STEPPER_DEADLINE_MISS_CHANNEL.swap(NO_STEPPER_CHANNEL, Relaxed);
                let stop_channel = if deadline_channel == NO_STEPPER_CHANNEL {
                    0
                } else {
                    deadline_channel
                };
                stepper_stop_from_p1(stop_channel);
                {
                    let s = driver.runtime.dispatch_state(tmc_state(), stepper_state());
                    *s.motor_enabled = false;
                    *s.calibrated = false;
                }
                enable_pin.set_low();

                let status_flags = tmc
                    .read_register_retried(foci_tmc4671::registers::STATUS_FLAGS)
                    .await
                    .map_err(|_| ());
                let flags = status_flags.unwrap_or(0);
                record_stepper_event(
                    StepperEventReason::TmcDisableSignal,
                    0,
                    read_tim5(),
                    UNKNOWN_QUEUE_LEN,
                    flags,
                    0,
                );
                #[cfg(feature = "trace")]
                crate::usb_trace::request_event(
                    foci_firmware::trace::events::EVENT_LEVEL_ERROR,
                    foci_firmware::trace::events::SUBSYSTEM_SAFETY_SHUTDOWN,
                    crate::usb_trace::EVENT_STATUS_INTERRUPT,
                    [flags as i32, 0, 0, 0],
                    1,
                );
                match status_flags {
                    Ok(flags) => {
                        defmt::error!("TMC4671 emergency disable: STATUS={:#010x}", flags)
                    }
                    Err(_) => defmt::error!("TMC4671 emergency disable: STATUS read failed"),
                }
                let status_pin_asserted = status_pin.is_high();
                tmc_state()
                    .status_pin_asserted
                    .store(status_pin_asserted, Relaxed);
                if status_pin_asserted || TMC_DISABLE_SIGNAL.signaled() {
                    defmt::error!("TMC4671 fault still reported after containment; latch kept");
                } else {
                    FAULT_ACTIVE.store(false, Relaxed);
                }
                let fault =
                    foci_firmware::tmc_control::classify_emergency_disable_fault(status_flags);
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        match fault {
                            foci_firmware::tmc_control::EmergencyDisableFault::AdcCurrentClipped => {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "TMC4671: ADC current clipped (overcurrent)",
                                    read_tim5(),
                                );
                            }
                            foci_firmware::tmc_control::EmergencyDisableFault::StatusReadFailed => {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "TMC4671: STATUS_FLAGS read failed",
                                    read_tim5(),
                                );
                            }
                            foci_firmware::tmc_control::EmergencyDisableFault::Generic => {
                                if deadline_channel != NO_STEPPER_CHANNEL {
                                    foci_firmware::dispatch_shutdown!(
                                        foci_firmware::shutdown::ShutdownReason::StepperTooFarInPast,
                                        read_tim5(),
                                        &mut crate::_ankyra_config::Sender
                                    );
                                } else {
                                    ::ankyra::klipper_shutdown_from!(
                                        &mut crate::_ankyra_config::Sender,
                                        "TMC4671 fault",
                                        read_tim5(),
                                    );
                                }
                            }
                        }
                    }
                });
                continue;
            }

            if matches!(
                driver.mode(),
                ChannelMode::Commissioning | ChannelMode::Restore
            ) {
                let exit = crate::tmc_control::effects::classify_commission_wait_exit(
                    SHUTDOWN_ACTIVE.load(Relaxed),
                    TMC_QUEUE_FLUSH.load(Relaxed),
                    TMC_DISABLE_SIGNAL.signaled(),
                    FAULT_ACTIVE.load(Relaxed),
                    crate::tmc_control::effects::lease_expired(read_tim5()),
                    tmc_state().cancel_requested.load(Relaxed),
                );
                if let Some(exit) = exit {
                    if !matches!(
                        exit,
                        crate::tmc_control::effects::CommissionWaitExit::Cancelled
                    ) {
                        driver.runtime.request_restore_cancel();
                    }
                    if matches!(
                        exit,
                        crate::tmc_control::effects::CommissionWaitExit::EmergencyDisable
                    ) {
                        enable_pin.set_low();
                    }
                    continue 'tmc_loop;
                }
            }
            scheduler.set_mode(0, driver.mode(), now_us());
            scheduler.set_maintenance_arm(MaintenanceArm::Armed, now_us());

            let control_sleep = Mono::delay_until(fugit::Instant::<u64, 1, 1_000_000>::from_ticks(
                scheduler
                    .earliest_due_us()
                    .unwrap_or_else(|| now_us() + 1_000),
            ));
            let selected = select(rx.recv(), control_sleep).await;

            if let Either::First(Ok(req)) = selected {
                let commissioning = matches!(
                    driver.mode(),
                    ChannelMode::Commissioning | ChannelMode::Restore
                );
                let disposition = if commissioning {
                    crate::tmc_control::effects::commission_wait_request_disposition(&req)
                } else {
                    foci_firmware::tmc_control::CommissionWaitRequestDisposition::Cancel
                };
                use foci_firmware::tmc_control::CommissionWaitRequestDisposition;
                match disposition {
                    CommissionWaitRequestDisposition::Cancel => {
                        if commissioning {
                            enable_pin.set_low();
                            tmc_state().commission_in_flight.store(false, Relaxed);
                        }
                        let outcome = handle_tmc_control_request(
                            req,
                            tmc,
                            enable_pin,
                            &mut driver.runtime,
                            &mut driver.commission_trace_engine_seq,
                            &mut *integral_candidates,
                        )
                        .await;
                        if let Some(reason) = outcome.shutdown {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    crate::tmc_control::effects::emit_shutdown(reason);
                                }
                            });
                        }
                        if outcome.should_refresh_velocity_scale {
                            refresh_velocity_scale(&mut driver.runtime);
                        }
                        if !commissioning && matches!(driver.mode(), ChannelMode::Commissioning) {
                            scheduler.set_commission_due(0, Some(now_us()));
                        }
                        if commissioning {
                            driver.runtime.request_restore_cancel();
                            continue 'tmc_loop;
                        }
                        if matches!(outcome.tail, TmcControlRequestTail::SkipTickTail) {
                            continue 'tmc_loop;
                        }
                    }
                    CommissionWaitRequestDisposition::RejectBusy => {
                        let outcome = handle_commission_wait_reject(req);
                        if let Some(reason) = outcome.shutdown {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    crate::tmc_control::effects::emit_shutdown(reason);
                                }
                            });
                        }
                        if outcome.should_refresh_velocity_scale {
                            refresh_velocity_scale(&mut driver.runtime);
                        }
                    }
                    CommissionWaitRequestDisposition::DropBusy => {}
                }
                scheduler.set_mode(0, driver.mode(), now_us());
            }

            let now = now_us();
            let homing_state = HOMING_STATE.load(Relaxed);
            let Some(unit) = scheduler.pick(now, homing_state) else {
                continue;
            };
            let snapshot = scheduler
                .maintenance_group()
                .map_or(homing_state, |g| g.homing_snapshot());
            #[cfg(feature = "trace")]
            let engine_tick_deadline =
                Mono::now() + fugit::Duration::<u64, 1, 1_000_000>::millis(1);
            let unit_start = cortex_m::peripheral::DWT::cycle_count();
            let outcome = if unit.class == UnitClass::MaintenanceFinalize {
                foci_firmware::tmc_control::TmcControlRuntime::advance_homing_state_after_channels(
                    foci_firmware::tmc_control::TmcControlHomingRefs::new(
                        &HOMING_STATE,
                        &HOMING_TRIGGERED,
                        &HOMING_REENTER_PENDING,
                    ),
                    snapshot,
                    group_all_ready,
                );
                group_all_ready = true;
                None
            } else {
                let mut backend = openffboard_backend(tmc, enable_pin);
                let integral_candidate = integral_candidates
                    .slot_mut(0)
                    .expect("OpenFFBoard channel 0 exists");
                let ctx_unit = crate::tmc_control::board::drive_ctx(
                    now,
                    snapshot,
                    status_pin_asserted,
                    &tmc_control_config,
                    integral_candidate,
                );
                Some(driver.drive(unit, &mut backend, ctx_unit).await)
            };
            let unit_cycles = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(unit_start);
            scheduler.record(now_us(), unit_cycles);
            let Some(outcome) = outcome else {
                continue;
            };

            match outcome {
                UnitOutcome::Idle => {}
                UnitOutcome::Reschedule { fast_period_us } => {
                    scheduler.set_fast_period(0, fast_period_us, now_us());
                }
                UnitOutcome::Ready => {
                    crate::tmc_control::board::enable_status_exti();
                    scheduler.set_mode(0, ChannelMode::Ready, now_us());
                }
                UnitOutcome::Parked(reason) => {
                    scheduler.set_mode(0, ChannelMode::Parked, now_us());
                    use crate::tmc_control::board::ParkShutdown;
                    let park = crate::tmc_control::board::park_shutdown(reason);
                    let clock = read_tim5();
                    ctx.shared.app_state.lock(|state| {
                        if !crate::firmware_ctx::enter_shutdown(state) {
                            return;
                        }
                        match park {
                            ParkShutdown::SpiFailedAtBoot => {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "TMC4671 SPI failed at boot",
                                    clock,
                                );
                            }
                            ParkShutdown::ChipIdMismatch => {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "TMC4671 chip ID mismatch",
                                    clock,
                                );
                            }
                            ParkShutdown::PiRepresentationFailedAtBoot => {
                                ::ankyra::klipper_shutdown_from!(
                                    &mut crate::_ankyra_config::Sender,
                                    "TMC4671 PI representation failed at boot",
                                    clock,
                                );
                            }
                            ParkShutdown::Reason(shutdown_reason) => {
                                foci_firmware::dispatch_shutdown!(
                                    shutdown_reason,
                                    clock,
                                    &mut crate::_ankyra_config::Sender
                                );
                            }
                        }
                    });
                }
                UnitOutcome::Shutdown(reason) => {
                    ctx.shared.app_state.lock(|state| {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            crate::tmc_control::effects::emit_shutdown(reason);
                        }
                    });
                }
                UnitOutcome::Maintenance { homing } => {
                    group_all_ready &= matches!(
                        homing,
                        foci_firmware::tmc_control::HomingMaintenanceResult::ReadyToAdvance
                    );
                    if matches!(
                        homing,
                        foci_firmware::tmc_control::HomingMaintenanceResult::FailClosed
                    ) {
                        tmc_state().homing_clamp_active.store(false, Relaxed);
                        tmc_state().stall_latched.store(false, Relaxed);
                        driver.runtime.clear_homing_runtime();
                        TMC_DISABLE_SIGNAL.signal(());
                    }
                    #[cfg(feature = "trace")]
                    {
                        let trace_tick = driver.runtime.advance_trace_tick();
                        let inputs =
                            foci_firmware::trace::sampling::TraceSampleInputs::from_runtime_trace(
                                driver.runtime.trace_state(),
                                trace_tick,
                            );
                        crate::usb_trace::sample_tick(
                            tmc,
                            crate::usb_trace::TraceSampleContext {
                                motor_enabled: driver.runtime.is_motor_enabled(),
                            },
                            inputs,
                        )
                        .await;
                        crate::usb_trace::emit_status_frames(
                            tmc,
                            trace_tick.control_tick,
                            trace_tick.status_detail_due,
                        )
                        .await;
                        crate::usb_trace::request_spi_duration_event_if_ready();
                        if trace_tick.metadata_due {
                            crate::usb_trace::emit_metadata();
                        }
                    }
                }
                UnitOutcome::Audit(result) => {
                    match result {
                        AuditResult::Clean | AuditResult::Warning { .. } => {}
                        AuditResult::Critical { .. }
                        | AuditResult::PinMismatch { .. }
                        | AuditResult::ReadFailed => {
                            FAULT_ACTIVE.store(true, Relaxed);
                            TMC_DISABLE_SIGNAL.signal(());
                        }
                    }
                    let fast = scheduler.fast_stats(0);
                    let commission = scheduler.commission_stats(0);
                    let maintenance = scheduler.maintenance_stats();
                    let audit = scheduler.audit_stats();
                    let arm = scheduler.maintenance_arm_stats();
                    defmt::info!(
                        "sched ch0 fast req={} svc={} miss={} late={}us cyc={} | commission req={} svc={} late={}us cyc={} | maint svc={} miss={} late={}us | audit svc={} miss={} | maint arm={:?} a2d={} d2a={}",
                        fast.requested,
                        fast.serviced,
                        fast.missed,
                        fast.max_lateness_us,
                        fast.max_unit_cycles,
                        commission.requested,
                        commission.serviced,
                        commission.max_lateness_us,
                        commission.max_unit_cycles,
                        maintenance.serviced,
                        maintenance.missed,
                        maintenance.max_lateness_us,
                        audit.serviced,
                        audit.missed,
                        arm.state,
                        arm.armed_to_disarmed,
                        arm.disarmed_to_armed,
                    );
                }
                UnitOutcome::Commission { event, next } => {
                    if let Some(event) = event {
                        match event {
                            foci_firmware::commissioning::schedule::CommissionEvent::PhaseChanged {
                                oid,
                                phase_id,
                                ..
                            } => emit_phase_reply(oid, phase_id),
                            foci_firmware::commissioning::schedule::CommissionEvent::VelocityIntegral {
                                oid,
                                event,
                            } => crate::ankyra::replies::emit_velocity_integral_event(oid, event),
                            foci_firmware::commissioning::schedule::CommissionEvent::BreakawayCampaign {
                                oid,
                                event,
                            } => crate::ankyra::replies::emit_breakaway_campaign_event(oid, event),
                            foci_firmware::commissioning::schedule::CommissionEvent::VelocityIntegralPlan {
                                oid,
                                run_sequence,
                                plan,
                            } => crate::ankyra::replies::emit_velocity_integral_plan(
                                oid,
                                run_sequence,
                                plan,
                            ),
                        }
                    }
                    #[cfg(feature = "trace")]
                    if driver.commission_trace_phase
                        == Some((
                            foci_firmware::commissioning::types::EngineKind::Commission,
                            foci_firmware::autotune::CommissionPhase::ClosedLoopActivation,
                        ))
                    {
                        let trace_tick = driver.runtime.advance_trace_sample_tick();
                        crate::usb_trace::sample_commission_closed_loop_activation(
                            tmc,
                            driver.runtime.is_motor_enabled(),
                            trace_tick,
                            || {
                                engine_tick_deadline
                                    .checked_duration_since(Mono::now())
                                    .map(|d| d.to_micros())
                                    .unwrap_or(0)
                            },
                            || Mono::now() > engine_tick_deadline,
                        )
                        .await;
                    }
                    match next {
                        CommissionNext::PollNow => scheduler.set_commission_due(0, Some(now_us())),
                        CommissionNext::WaitUntil(deadline_us) => {
                            scheduler.set_commission_due(0, Some(deadline_us))
                        }
                        CommissionNext::Yield => {
                            scheduler.note_commission_yield(0);
                            scheduler.set_commission_due(0, Some(now_us()));
                        }
                    }
                    scheduler.set_mode(0, driver.mode(), now_us());
                }
                UnitOutcome::CommissionTerminal { reply, actions } => {
                    {
                        let mut state = driver.runtime.dispatch_state(tmc_state(), stepper_state());
                        apply_stepper_post_action(actions.stepper, &mut state);
                    }
                    foci_firmware::diagnostic::mark_p1_reply_construction_start();
                    emit_terminal_reply(0, reply);
                    foci_firmware::diagnostic::mark_p1_reply_construction_end();
                    defmt::info!(
                        "commissioning terminal max tick cycles: {}",
                        scheduler.commission_stats(0).max_unit_cycles
                    );
                    scheduler.set_commission_due(0, None);
                    scheduler.set_mode(0, driver.mode(), now_us());
                }
            }
        }
    }

    impl StepperStopCheckBoard for crate::firmware_ctx::OpenFfboardStepperBoard {
        fn stop_check_index(&self, channel: u8) -> Option<usize> {
            (channel == 0).then_some(0)
        }

        fn clear_step_timer(
            &mut self,
            state: &mut foci_firmware::context::trsync::TriggerDomainState,
            stepper_idx: usize,
            _channel: u8,
        ) {
            state.scheduler.cancel(|kind| {
                matches!(
                    kind,
                    foci_klipper::scheduler::SchedulerEventKind::Stepper(0)
                )
            });
            self.set_timer_active(state, stepper_idx, false);
            reprogram_timer(state);
        }
    }

    impl StepperArmCheckBoard for crate::firmware_ctx::OpenFfboardStepperBoard {
        fn arm_check_index(&self, channel: u8) -> Option<usize> {
            (channel == 0).then_some(0)
        }

        fn motor_armed(&self, _channel: u8) -> bool {
            tmc_state()
                .motor_armed
                .load(core::sync::atomic::Ordering::Relaxed)
        }

        fn queue_has_moves(&self, _channel: u8) -> bool {
            !crate::state::stepper::STEP_QUEUE_CHANNEL.is_empty()
        }

        fn arm_check_timer(
            &mut self,
            state: &mut foci_firmware::context::trsync::TriggerDomainState,
            stepper_idx: usize,
            channel: u8,
        ) {
            let now = read_tim5();
            crate::state::stepper_diag::record_timer_activation(channel, now);
            state.scheduler.cancel(|kind| {
                matches!(
                    kind,
                    foci_klipper::scheduler::SchedulerEventKind::Stepper(0)
                )
            });
            insert_trigger_event_unmeasured(
                state,
                foci_klipper::scheduler::SchedulerEvent {
                    wake_clock: now.wrapping_add(1),
                    kind: foci_klipper::scheduler::SchedulerEventKind::Stepper(0),
                },
            );
            self.set_timer_active(state, stepper_idx, true);
            reprogram_timer(state);
        }
    }

    #[task(priority = 2)]
    async fn stepper_reconcile(_ctx: stepper_reconcile::Context) {
        loop {
            STEPPER_RECONCILE_WAKE.wait().await;
            loop {
                let pending = STEPPER_RECONCILE_PENDING.take_pending();
                if pending.is_empty() {
                    break;
                }
                with_trigger_domain(|td| {
                    let mut board = crate::firmware_ctx::OpenFfboardStepperBoard;
                    drain_stepper_reconciliation(td.state_mut(), &mut board, pending);
                });
            }
        }
    }

    const TIM5_SCHEDULER_REPEAT_TICKS: u32 = CLOCK_FREQ / 10_000;
    const TIM5_SCHEDULER_DEFER_TICKS: u32 = CLOCK_FREQ / 200_000;

    fn record_schedule_result(result: foci_klipper::scheduler::ScheduleResult) -> bool {
        match result {
            foci_klipper::scheduler::ScheduleResult::Inserted { new_earliest } => new_earliest,
            foci_klipper::scheduler::ScheduleResult::Full => false,
        }
    }

    fn insert_trigger_event_unmeasured(
        state: &mut foci_firmware::context::trsync::TriggerDomainState,
        event: foci_klipper::scheduler::SchedulerEvent,
    ) -> bool {
        record_schedule_result(state.scheduler.try_schedule(event))
    }

    fn insert_trigger_event(
        td: &mut TriggerDomain,
        event: foci_klipper::scheduler::SchedulerEvent,
    ) -> bool {
        record_schedule_result(td.state_mut().scheduler.try_schedule(event))
    }

    fn arm_stepper_timer_event(
        td: &mut TriggerDomain,
        stepper_idx: usize,
        clock: u32,
        slot: Option<foci_klipper::scheduler::ScheduledSlot>,
    ) {
        if let Some(slot) = slot {
            td.state_mut().scheduler.requeue_slot(slot, clock);
        } else {
            insert_trigger_event(
                td,
                foci_klipper::scheduler::SchedulerEvent {
                    wake_clock: clock,
                    kind: foci_klipper::scheduler::SchedulerEventKind::Stepper(stepper_idx),
                },
            );
        }
    }

    #[inline(always)]
    fn spin_tim5_ticks(start: u32, ticks: u32) {
        while read_tim5().wrapping_sub(start) < ticks {
            core::hint::spin_loop();
        }
    }

    #[inline(always)]
    fn emit_logical_step_event(event: foci_klipper::stepper::LogicalStepEvent) {
        let gpio_a = embassy_stm32::pac::GPIOA;
        let gpio_b = embassy_stm32::pac::GPIOB;

        gpio_b.bsrr().write(|w| w.set_br(11, true));
        if event.dir_setup_required {
            gpio_a.bsrr().write(|w| {
                if event.direction {
                    w.set_bs(0, true);
                } else {
                    w.set_br(0, true);
                }
            });
            let setup_start = read_tim5();
            spin_tim5_ticks(setup_start, crate::state::stepper::STEP_DIR_SETUP_TICKS);
        }

        crate::state::stepper::assert_physical_pulse_budget(event.physical_pulses);
        for _ in 0..event.physical_pulses {
            gpio_b.bsrr().write(|w| w.set_bs(11, true));
            let high_start = read_tim5();
            spin_tim5_ticks(high_start, crate::state::stepper::STEP_HALF_PERIOD_TICKS);
            gpio_b.bsrr().write(|w| w.set_br(11, true));
            let low_start = read_tim5();
            spin_tim5_ticks(low_start, crate::state::stepper::STEP_HALF_PERIOD_TICKS);
        }
    }

    fn process_stepper_timer(
        td: &mut TriggerDomain,
        stepper_idx: usize,
        slot: Option<foci_klipper::scheduler::ScheduledSlot>,
    ) {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::stepper::StepEventResult;

        if stepper_idx != 0 {
            return;
        }
        let mut stepper_slot = slot;
        let gpio_b = embassy_stm32::pac::GPIOB;
        gpio_b.bsrr().write(|w| w.set_br(11, true));
        if FAULT_ACTIVE.load(Relaxed) || !tmc_state().motor_armed.load(Relaxed) {
            set_timer_active(td.state_mut(), 0, false);
            stepper_state().interval.store(0, Relaxed);
            stepper_state().interval_add.store(0, Relaxed);
            tmc_state().pid_error_sums_clear.store(true, Relaxed);
            return;
        }
        // SAFETY: ISR stepper state is only accessed from this P3 ISR or from
        // stop paths that first gate/cancel the stepper timer.
        let stepper = unsafe { isr_stepper(0).expect("OpenFFBoard channel 0 exists") };

        let gpio_a = embassy_stm32::pac::GPIOA;

        loop {
            if !stepper.has_active_segment() {
                // SAFETY: P3 is the sole consumer while motor_armed is true.
                let loaded_steps =
                    match unsafe { crate::state::stepper::STEP_QUEUE_CHANNEL.isr_dequeue() } {
                        Some(m) => {
                            let loaded_steps = u32::from(m.count);
                            let interval_add = i32::from(m.add);
                            let old_direction = stepper.dir_state();
                            #[cfg(feature = "trace")]
                            if let Some((old_class, new_class)) = stepper.load_move(m) {
                                crate::usb_trace::request_event(
                                    foci_firmware::trace::events::EVENT_LEVEL_INFO,
                                    foci_firmware::trace::events::SUBSYSTEM_STEP_TIMER,
                                    foci_firmware::trace::events::EVENT_PHASE_BOUNDARY,
                                    [0, old_class as i32, new_class as i32, 0],
                                    3,
                                );
                            }
                            #[cfg(not(feature = "trace"))]
                            stepper.load_move(m);
                            let dir_changed = old_direction != stepper.dir_state();
                            stepper_state()
                                .interval
                                .store(stepper.current_interval(), Relaxed);
                            stepper_state().interval_add.store(interval_add, Relaxed);
                            let dir_val = if stepper.dir_state() {
                                embassy_stm32::pac::gpio::vals::Odr::HIGH
                            } else {
                                embassy_stm32::pac::gpio::vals::Odr::LOW
                            };
                            gpio_a.odr().modify(|w| w.set_odr(0, dir_val));
                            if dir_changed {
                                tmc_state().feedforward_reset.store(true, Relaxed);
                            }
                            loaded_steps
                        }
                        None => {
                            stepper.mark_idle();
                            crate::state::stepper_diag::record_queue_empty(0);
                            if HOMING_STATE.load(Relaxed) != 0 {
                                record_stepper_event(
                                    StepperEventReason::QueueEmpty,
                                    0,
                                    read_tim5(),
                                    0,
                                    0,
                                    0,
                                );
                            }
                            stepper_state().interval.store(0, Relaxed);
                            stepper_state().interval_add.store(0, Relaxed);
                            tmc_state().pid_error_sums_clear.store(true, Relaxed);
                            set_timer_active(td.state_mut(), 0, false);
                            return;
                        }
                    };

                let now = read_tim5();
                let scheduled = stepper.next_step_clock();
                #[cfg(feature = "trace")]
                crate::usb_trace::record_step_timing(scheduled, now);
                match stepper.check_deadline(scheduled, now, CLOCK_FREQ) {
                    Ok(compare) => {
                        crate::state::stepper_diag::record_loaded_segment_timing(
                            0,
                            loaded_steps,
                            now,
                            scheduled,
                            compare,
                        );
                        arm_stepper_timer_event(td, 0, compare, stepper_slot.take());
                        return;
                    }
                    Err(_) => {
                        crate::state::stepper_diag::record_loaded_segment(0, loaded_steps);
                        crate::state::stepper_diag::record_missed_deadline(0);
                        #[cfg(feature = "trace")]
                        crate::usb_trace::TRACE_TIMING
                            .missed_or_deferred_steps
                            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                        record_stepper_event(
                            StepperEventReason::MissedDeadlineLoad,
                            0,
                            now,
                            UNKNOWN_QUEUE_LEN,
                            scheduled,
                            now,
                        );
                        set_timer_active(td.state_mut(), 0, false);
                        STEPPER_DEADLINE_MISS_CHANNEL.store(0, Relaxed);
                        TMC_DISABLE_SIGNAL.signal(());
                        return;
                    }
                }
            }

            if step_held_for_homing(stepper.next_step_clock()) {
                arm_stepper_timer_event(
                    td,
                    0,
                    read_tim5().wrapping_add(CLOCK_FREQ / 1000),
                    stepper_slot.take(),
                );
                set_timer_active(td.state_mut(), 0, true);
                reprogram_timer(td.state_mut());
                return;
            }

            let result = stepper.step_event();
            let event = result.event();
            emit_logical_step_event(event);

            let now = read_tim5();
            crate::state::stepper::publish_logical_event(stepper, event);
            let logical_delta = if event.direction { 1 } else { -1 };
            stepper_state().position.fetch_add(logical_delta, Relaxed);
            stepper_state().direction.store(event.direction, Relaxed);
            stepper_state()
                .interval
                .store(stepper.current_interval(), Relaxed);
            crate::state::stepper_diag::record_executed_step(
                0,
                event.direction,
                event.physical_pulses,
                now,
            );

            match result {
                StepEventResult::ScheduleNext { clock, .. } => {
                    #[cfg(feature = "trace")]
                    crate::usb_trace::record_step_timing(clock, now);
                    arm_stepper_timer_event(td, 0, clock, stepper_slot.take());
                    return;
                }
                StepEventResult::NeedMove { .. } => {}
            }
        }
    }

    fn process_trigger_timer(
        td: &mut TriggerDomain,
        kind: foci_klipper::scheduler::SchedulerEventKind,
        wake_clock: u32,
        need_report: &mut bool,
        stepper_slot: Option<foci_klipper::scheduler::ScheduledSlot>,
    ) {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::scheduler::SchedulerEventKind;

        match kind {
            SchedulerEventKind::Stepper(stepper_idx) => {
                process_stepper_timer(td, stepper_idx, stepper_slot);
            }
            SchedulerEventKind::EndstopSample(endstop_idx) => {
                use foci_klipper::endstop::{EndstopPhase, EndstopSampleResult};
                let pin_id = ENDSTOP_PIN_IDS[endstop_idx].load(Relaxed);
                let read_pin = || -> bool {
                    match crate::ankyra::enumerations::Pins::try_from(pin_id) {
                        Ok(crate::ankyra::enumerations::Pins::STALL0) => {
                            tmc_state().stall_latched.load(Relaxed)
                        }
                        _ if pin_id != 0xFF => gpio_read_pin(pin_id),
                        _ => false,
                    }
                };

                let phase = td.state().endstops[endstop_idx].phase();
                let result = match phase {
                    EndstopPhase::Scanning => {
                        td.state_mut().endstops[endstop_idx].scan(read_pin(), wake_clock)
                    }
                    EndstopPhase::Oversampling => {
                        td.state_mut().endstops[endstop_idx].oversample(read_pin(), wake_clock)
                    }
                };

                match result {
                    EndstopSampleResult::EnterOversamplingNow => {
                        match td.state_mut().endstops[endstop_idx]
                            .oversample(read_pin(), wake_clock)
                        {
                            EndstopSampleResult::Triggered { trsync_oid, reason } => {
                                td.trigger(trsync_oid, reason, read_tim5());
                                *need_report = true;
                            }
                            EndstopSampleResult::Reschedule { next_clock } => {
                                insert_trigger_event(
                                    td,
                                    foci_klipper::scheduler::SchedulerEvent {
                                        wake_clock: next_clock,
                                        kind: SchedulerEventKind::EndstopSample(endstop_idx),
                                    },
                                );
                            }
                            _ => {}
                        }
                    }
                    EndstopSampleResult::Triggered { trsync_oid, reason } => {
                        td.trigger(trsync_oid, reason, read_tim5());
                        *need_report = true;
                    }
                    EndstopSampleResult::Reschedule { next_clock } => {
                        insert_trigger_event(
                            td,
                            foci_klipper::scheduler::SchedulerEvent {
                                wake_clock: next_clock,
                                kind: SchedulerEventKind::EndstopSample(endstop_idx),
                            },
                        );
                    }
                    EndstopSampleResult::Inactive => {}
                }
            }
            SchedulerEventKind::TrSyncExpire(trsync_idx) => {
                let oid = td
                    .state()
                    .trsync_by_oid
                    .iter()
                    .find(|&(_, &idx)| idx == trsync_idx)
                    .map(|(&oid, _)| oid);
                if let Some(oid) = oid {
                    let expire_reason = td.state().trsync[trsync_idx].expire_reason();
                    td.trigger(oid, expire_reason, read_tim5());
                    *need_report = true;
                }
            }
            SchedulerEventKind::TrSyncReport(trsync_idx) => {
                let oid = td
                    .state()
                    .trsync_by_oid
                    .iter()
                    .find(|&(_, &idx)| idx == trsync_idx)
                    .map(|(&oid, _)| oid);
                if let Some(oid) = oid
                    && td.state().trsync[trsync_idx].can_trigger()
                {
                    let now = read_tim5();
                    td.state_mut().trsync[trsync_idx].set_liveness_report(oid, now);
                    *need_report = true;
                    let report_ticks = td.state().trsync[trsync_idx].report_ticks();
                    if report_ticks > 0 {
                        let next_report = td.state().trsync[trsync_idx]
                            .report_clock()
                            .wrapping_add(report_ticks);
                        td.state_mut().trsync[trsync_idx].set_report_clock(next_report);
                        insert_trigger_event(
                            td,
                            foci_klipper::scheduler::SchedulerEvent {
                                wake_clock: next_report,
                                kind: SchedulerEventKind::TrSyncReport(trsync_idx),
                            },
                        );
                    }
                }
            }
        }
    }

    fn defer_tim5_scheduler(td: &mut TriggerDomain) {
        if td.state().scheduler.is_empty() {
            disarm_timer(td.state_mut());
            return;
        }
        #[cfg(feature = "trace")]
        crate::usb_trace::TRACE_TIMING
            .missed_or_deferred_steps
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        arm_timer_at(
            td.state_mut(),
            read_tim5().wrapping_add(TIM5_SCHEDULER_DEFER_TICKS),
        );
    }

    fn dispatch_tim5_scheduler(td: &mut TriggerDomain, need_report: &mut bool) {
        let dispatch_start = read_tim5();
        loop {
            let now = read_tim5();
            let slot = td.state_mut().scheduler.pop_due_slot(now);
            if let Some(slot) = slot {
                let event = slot.event();
                let stepper_slot = match event.kind {
                    foci_klipper::scheduler::SchedulerEventKind::Stepper(_) => Some(slot),
                    _ => None,
                };
                if let Some((channel, StepDeadlineStatus::Fatal { .. })) =
                    crate::state::stepper::scheduler_step_deadline_status(
                        event.kind,
                        event.wake_clock,
                        now,
                    )
                {
                    let channel = channel as u8;
                    crate::state::stepper_diag::record_missed_deadline(channel);
                    record_stepper_event(
                        StepperEventReason::MissedDeadlineStep,
                        channel,
                        now,
                        UNKNOWN_QUEUE_LEN,
                        event.wake_clock,
                        now,
                    );
                    STEPPER_DEADLINE_MISS_CHANNEL.store(channel, Relaxed);
                    disarm_timer(td.state_mut());
                    set_timer_active(td.state_mut(), channel as usize, false);
                    TMC_DISABLE_SIGNAL.signal(());
                    return;
                }
                process_trigger_timer(td, event.kind, event.wake_clock, need_report, stepper_slot);
                let elapsed = read_tim5().wrapping_sub(dispatch_start) as i32;
                if elapsed >= TIM5_SCHEDULER_REPEAT_TICKS as i32 {
                    defer_tim5_scheduler(td);
                    return;
                }
                continue;
            }

            let next_clock = td.state().scheduler.earliest_deadline();
            let Some(next_clock) = next_clock else {
                disarm_timer(td.state_mut());
                return;
            };
            let now = read_tim5();
            let compare = foci_klipper::scheduler::timer_arm_clock(
                now,
                next_clock,
                TRIGGER_TIMER_MIN_LEAD_TICKS,
            );
            arm_timer_at(td.state_mut(), compare);
            return;
        }
    }

    #[task(binds = TIM5, priority = 3)]
    fn step_timer_irq(_ctx: step_timer_irq::Context) {
        use core::sync::atomic::Ordering::Relaxed;

        let tim5 = embassy_stm32::pac::TIM5;
        let sr = tim5.sr().read();
        let dier = tim5.dier().read();
        let cc1_pending = sr.ccif(0) && dier.ccie(0);
        let cc2_pending = sr.ccif(1) && dier.ccie(1);
        if !cc1_pending && !cc2_pending {
            return;
        }
        tim5.sr().modify(|w| {
            w.set_ccif(0, false);
            w.set_ccif(1, false);
        });

        let mut need_report = false;
        with_trigger_domain(|td| {
            dispatch_tim5_scheduler(td, &mut need_report);
        });
        if need_report {
            REPORT_PENDING.store(true, Relaxed);
            report_task::spawn().ok();
        }
    }

    #[task(binds = OTG_FS, priority = 3)]
    fn otg_fs_irq(_ctx: otg_fs_irq::Context) {
        crate::interrupts::otg_fs_irq();
    }

    #[task(binds = EXTI9_5, priority = 3)]
    fn tmc_status_irq(_ctx: tmc_status_irq::Context) {
        crate::interrupts::tmc_status_irq(&FAULT_ACTIVE, &TMC_DISABLE_SIGNAL);
    }

    #[task(priority = 2, shared = [app_state])]
    async fn usb_task(mut ctx: usb_task::Context) {
        let mut backend = crate::usb_task::OpenffUsbBackend::new(
            |f| ctx.shared.app_state.lock(f),
            &REPORT_PENDING,
            &USB_PUMP_EMITTER,
            &USB_PUMP_COUNTERS,
        );
        loop {
            foci_firmware::usb_pump::run_once(&mut backend, &USB_PUMP_COUNTERS).await;
        }
    }

    #[task(priority = 2)]
    async fn report_task(_ctx: report_task::Context) {
        crate::heartbeat::emit_trigger_reports(
            |snaps| {
                with_trigger_domain(|td| {
                    snaps.extend(
                        td.state_mut()
                            .trsync
                            .iter_mut()
                            .filter_map(|ts| ts.take_report()),
                    );
                });
            },
            &REPORT_PENDING,
        );
    }

    #[task(priority = 2)]
    async fn stepper_diag_task(_ctx: stepper_diag_task::Context) {
        crate::heartbeat::emit_stepper_diagnostics(&STEPPER_DIAG_PENDING);
    }

    #[task(priority = 2)]
    async fn trigger_drain_task(_ctx: trigger_drain_task::Context) {
        loop {
            crate::trigger_domain::TRIGGER_DRAIN_WAKE.wait().await;
            crate::trigger_domain::drain_pending_stop();
        }
    }

    #[task(priority = 1)]
    async fn stats(_ctx: stats::Context) {
        let mut accumulator = crate::heartbeat::StatsAccumulator::new(read_tim5());
        loop {
            Mono::delay(5000.millis()).await;
            crate::heartbeat::stats_tick(
                &mut accumulator,
                read_tim5(),
                &STATS_SEND_TIME,
                &STATS_SEND_TIME_HIGH,
            );
        }
    }

    #[task(priority = 2, local = [wdg])]
    async fn watchdog(ctx: watchdog::Context) {
        loop {
            Mono::delay(crate::heartbeat::WATCHDOG_INTERVAL_MS.millis()).await;
            crate::heartbeat::watchdog_tick(ctx.local.wdg);
        }
    }

    #[task(
        priority = 1,
        shared = [app_state],
        local = [adc_internal, temp_channel, mcu_temp_calibration]
    )]
    async fn analog_sampling(mut ctx: analog_sampling::Context) {
        loop {
            Mono::delay(1.millis()).await;
            let now = read_tim5();
            let shutdown_requested = crate::heartbeat::analog_sampling_tick(
                ctx.local.adc_internal,
                ctx.local.temp_channel,
                ctx.local.mcu_temp_calibration,
                now,
            );
            if shutdown_requested {
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        foci_firmware::dispatch_shutdown!(
                            foci_firmware::shutdown::ShutdownReason::AdcOutOfRange,
                            now,
                            &mut crate::_ankyra_config::Sender
                        );
                    }
                });
            }
        }
    }
}

foci_firmware::foci_ankyra_config! {
    transport = crate::transport::TRANSPORT_OUTPUT: crate::transport::BufferTransportOutput,
    context = &'ctx mut crate::firmware_ctx::FirmwareCtx<'ctx>,
    providers = [
        foci_firmware::ankyra::provider::FOCI_FIRMWARE_PROVIDER,
        crate::ankyra::provider::BOARD_PROVIDER,
    ],
    board_static_strings = [
        "TMC4671 PI representation failed at boot",
        "TMC4671: STATUS_FLAGS read failed",
    ],
    app = "FOCI",
    version = env!("FOCI_VERSION"),
    build_versions = env!("FOCI_BUILD_VERSIONS"),
    license = "MIT OR Apache-2.0",
}
