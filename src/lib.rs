//! A platform agnostic Rust driver for EMC2101, based on the
//! [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.

#![no_std]
pub(crate) mod fmt;

mod error;
use bounded_integer::BoundedU8;
pub use error::{Error, Result};

#[cfg(not(any(feature = "sync", feature = "async")))]
compile_error!("You should probably choose at least one of `sync` and `async` features.");

#[cfg(feature = "sync")]
use embedded_hal::i2c::ErrorType;
#[cfg(feature = "sync")]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::ErrorType as AsyncErrorType;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

use fugit::HertzU32;
use heapless::Vec;

/// EMC2101 sensor's I2C address.
pub const DEFAULT_ADDRESS: u8 = 0b1001100; // This is I2C address 0x4C

const EMC2101_PRODUCT_ID: u8 = 0x16;
const EMC2101R_PRODUCT_ID: u8 = 0x28;

/// EMC2101 sensor's Product.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Product {
    EMC2101,
    EMC2101R,
}

/// ADC Conversion Rates.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ConversionRate {
    Rate1_16Hz = 0,
    Rate1_8Hz = 1,
    Rate1_4Hz = 2,
    Rate1_2Hz = 3,
    Rate1Hz = 4,
    Rate2Hz = 5,
    Rate4Hz = 6,
    Rate8Hz = 7,
    Rate16Hz = 8,
    Rate32Hz = 9,
}

/// ADC Filter Levels.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FilterLevel {
    Disabled = 0,
    Level1 = 1,
    Level2 = 3,
}

/// Beta Compensation of the External Diode channel.
///
/// The fixed settings are named after the minimum beta of the transistor connected to the External
/// Diode channel. A factor set above the transistor beta can introduce measurement errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BetaCompensation {
    Min0_11 = 0,
    Min0_18 = 1,
    Min0_25 = 2,
    Min0_33 = 3,
    Min0_43 = 4,
    Min1_00 = 5,
    Min2_33 = 6,
    /// For a diode-connected transistor (such as the 2N3904) or a two-terminal diode.
    Disabled = 7,
    /// The optimal factor is detected at the beginning of every conversion (default).
    Auto = 8,
}

/// Converts a raw BetaCompensation Register value, such as one stored in a configuration.
/// As on the device, every value with the ENABLE bit set (0x08 to 0x0F) gives `Auto`.
impl TryFrom<u8> for BetaCompensation {
    type Error = Error<()>;

    fn try_from(value: u8) -> core::result::Result<Self, Self::Error> {
        Ok(match value {
            0 => Self::Min0_11,
            1 => Self::Min0_18,
            2 => Self::Min0_25,
            3 => Self::Min0_33,
            4 => Self::Min0_43,
            5 => Self::Min1_00,
            6 => Self::Min2_33,
            7 => Self::Disabled,
            8..=15 => Self::Auto,
            _ => return Err(Error::InvalidValue),
        })
    }
}

/// Base clock of the PWM driver, see `set_fan_pwm_advanced`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmClock {
    /// 360 kHz (power-on default).
    Base360kHz,
    /// 1.4 kHz.
    Base1400Hz,
    /// 360 kHz divided by PWM_D (1 to 255).
    Divided(BoundedU8<1, 255>),
}

/// Manual implementation of defmt 'Format' trait, since BoundedU8 does not implement it.
#[cfg(feature = "defmt")]
impl defmt::Format for PwmClock {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            PwmClock::Base360kHz => defmt::write!(fmt, "Base360kHz"),
            PwmClock::Base1400Hz => defmt::write!(fmt, "Base1400Hz"),
            PwmClock::Divided(pwm_d) => defmt::write!(fmt, "Divided({=u8})", pwm_d.get()),
        }
    }
}

/// Registers of the EMC2101 sensor.
#[cfg(any(feature = "async", feature = "sync"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Register {
    InternalTemperature = 0x00,
    ExternalTemperatureMSB = 0x01,
    Status = 0x02,
    Configuration = 0x03,
    ConversionRate = 0x04,
    InternalTempLimit = 0x05,
    ExternalTempLimitHigh = 0x07,
    ExternalTempLimitLow = 0x08,
    ExternalTemperatureForce = 0x0C,
    OneShot = 0x0F,
    ExternalTemperatureLSB = 0x10,
    AlertMask = 0x16,
    ExternalIdealityFactor = 0x17,
    BetaCompensation = 0x18,
    ExternalTempCriticalLimit = 0x19,
    ExternalTempCriticalHysteresis = 0x21,
    TachLSB = 0x46,
    TachMSB = 0x47,
    FanConfig = 0x4A,
    FanSetting = 0x4C,
    PWMFrequency = 0x4D,
    PWMFrequencyDivide = 0x4E,
    FanControlLUTHysteresis = 0x4F,
    FanControlLUTT1 = 0x50,
    FanControlLUTS1 = 0x51,
    AveragingFilter = 0xBF,
    ProductID = 0xFD,
}

#[cfg(any(feature = "async", feature = "sync"))]
impl From<Register> for u8 {
    fn from(r: Register) -> u8 {
        r as u8
    }
}

/// Device Satuts.
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    pub eeprom_error: bool,
    pub ext_diode_fault: bool,
    pub adc_busy: bool,
    pub temp_int_high: bool,
    pub temp_ext_high: bool,
    pub temp_ext_low: bool,
    pub temp_ext_critical: bool,
    pub tack_limit: bool,
}

impl From<u8> for Status {
    fn from(s: u8) -> Self {
        Self {
            eeprom_error: s & 0x20 == 0x20,
            ext_diode_fault: s & 0x04 == 0x04,
            adc_busy: s & 0x80 == 0x80,
            temp_int_high: s & 0x40 == 0x40,
            temp_ext_high: s & 0x10 == 0x10,
            temp_ext_low: s & 0x08 == 0x08,
            temp_ext_critical: s & 0x02 == 0x02,
            tack_limit: s & 0x01 == 0x01,
        }
    }
}

/// Look-up Table Level
#[derive(Debug, Clone, Copy, Default)]
pub struct Level {
    pub temp: BoundedU8<0, 127>,
    pub step: BoundedU8<0, 63>,
}

/// Manual implementation of defmt 'Format' trait, since BoundedU8 does not implement it.
#[cfg(feature = "defmt")]
impl defmt::Format for Level {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Level {{ temp: {=u8}, step: {=u8} }}",
            self.temp.get(),
            self.step.get(),
        );
    }
}

/// An EMC2101 sensor on the I2C bus `I`.
///
/// The address of the sensor will be `DEFAULT_ADDRESS` from this package,
/// unless there is some kind of special address translating hardware in use.
#[maybe_async_cfg2::maybe(
    sync(feature = "sync", self = "EMC2101"),
    async(feature = "async", keep_self)
)]
pub struct AsyncEMC2101<I> {
    i2c: I,
    address: u8,
    variant: Option<Product>,
    /// Whether the Fan Control Look-Up Table drives the fan (FanConfig[5] PROG cleared).
    /// None while unknown: a previous firmware may have left the Look-Up Table in use.
    lut_active: Option<bool>,
}

#[maybe_async_cfg2::maybe(
    sync(
        feature = "sync",
        self = "EMC2101",
        idents(AsyncI2c(sync = "I2c"), AsyncErrorType(sync = "ErrorType"))
    ),
    async(feature = "async", keep_self)
)]
impl<I: AsyncI2c + AsyncErrorType> AsyncEMC2101<I> {
    /// Initializes the EMC2101 driver.
    ///
    /// This consumes the I2C bus `I`. The address will almost always
    /// be `DEFAULT_ADDRESS` from this crate.
    ///
    /// All alerts are disabled, and the rest of the configuration is left as the device holds it.
    /// The EMC2101 keeps its registers across a reset of the host only, so settings left by a
    /// previous firmware (standby, fan driver mode, Look-up Table, ...) stay in effect until they
    /// are set again.
    pub async fn with_address(i2c: I, address: u8) -> Result<Self, I::Error> {
        let mut emc2101 = AsyncEMC2101 {
            i2c,
            address,
            variant: None,
            lut_active: None,
        };
        trace!("new");
        emc2101.variant = Some(emc2101.check_id().await?);
        // Disable all alerts interrupt, will be enable one by one calling monitor_xxx() functions.
        emc2101.write_reg(Register::AlertMask, 0xFF).await?;
        Ok(emc2101)
    }

    /// Initializes the EMC2101 driver at `DEFAULT_ADDRESS`, see `with_address`.
    pub async fn new(i2c: I) -> Result<Self, I::Error> {
        AsyncEMC2101::with_address(i2c, DEFAULT_ADDRESS).await
    }

    /// check_id asks the EMC2101 sensor to report its Product.
    async fn check_id(&mut self) -> Result<Product, I::Error> {
        trace!("check_id");
        match self.read_reg(Register::ProductID).await? {
            EMC2101_PRODUCT_ID => Ok(Product::EMC2101),
            EMC2101R_PRODUCT_ID => Ok(Product::EMC2101R),
            _ => Err(Error::InvalidID),
        }
    }

    /// set_standby put the device in low power standby mode, or back in operational mode.
    /// In standby mode the temperature monitor is disabled, and the temperature data is only
    /// updated by one_shot. FAN_STANDBY is not changed, and its default keeps the FAN output active.
    pub async fn set_standby(&mut self, standby: bool) -> Result<&mut Self, I::Error> {
        trace!("set_standby");
        if standby {
            // Set Configuration[6] STANDBY : Low power standby mode. In this mode, the Temperature
            // monitor is disabled.
            self.update_reg(Register::Configuration, 0b0100_0000, 0)
                .await?;
        } else {
            // Clear Configuration[6] STANDBY : Operational mode, monitoring temperatures, updating
            // FAN output.
            self.update_reg(Register::Configuration, 0, 0b0100_0000)
                .await?;
        }
        Ok(self)
    }

    /// one_shot start a single temperature conversion, in standby mode only.
    /// When the conversion is complete, the temperature data is updated, and so is the fan setting
    /// if the Look-up Table is used. Status adc_busy is set while the conversion runs.
    pub async fn one_shot(&mut self) -> Result<&mut Self, I::Error> {
        trace!("one_shot");
        // Writing to the OneShot Register initiates a one shot update of the temperature data, when
        // the temperature monitoring is in standby mode. Data is not relevant and is not stored.
        self.write_reg(Register::OneShot, 0).await?;
        Ok(self)
    }

    /// status gives the device current status.
    pub async fn status(&mut self) -> Result<Status, I::Error> {
        trace!("status");
        Ok(self.read_reg(Register::Status).await?.into())
    }

    /// configure_adc set the conversion rate in Hertz and the filter level.
    pub async fn configure_adc(
        &mut self,
        rate: ConversionRate,
        filter: FilterLevel,
    ) -> Result<&mut Self, I::Error> {
        trace!("configure_adc");
        // ConversionRate[3:0] : ADC conversion rate.
        self.write_reg(Register::ConversionRate, rate as u8).await?;
        // AveragingFilter[2:1] FILTER[1:0] : control the level of digital filtering
        // that is applied to the External Diode temperature measurements.
        let f_set: u8 = (filter as u8) << 1;
        let f_clr: u8 = !f_set & 0x06;
        self.update_reg(Register::AveragingFilter, f_set, f_clr)
            .await?;
        Ok(self)
    }

    /// force_temp_external force the external temperature value to a virtual value.
    /// When determining the position of the Fan Control Look-up Table, the contents
    /// of the ExternalTemperatureForce Register will be used instead of the measured
    /// External Diode temperature as normal.
    pub async fn force_temp_external(&mut self, value: i8) -> Result<&mut Self, I::Error> {
        trace!("force_temp_external");
        self.write_reg(Register::ExternalTemperatureForce, value as u8)
            .await?;
        // Set FanConfig[6] FORCE : the ExternalTemperatureForce Register is used.
        self.update_reg(Register::FanConfig, 0b0100_0000, 0).await?;
        Ok(self)
    }

    /// real_temp_external let the measured External Diode temperature be used to
    /// determine the position in the Fan Control Look-up Table.
    pub async fn real_temp_external(&mut self) -> Result<&mut Self, I::Error> {
        trace!("real_temp_external");
        // Clear FanConfig[6] FORCE : the ExternalTemperatureForce Register is not used.
        self.update_reg(Register::FanConfig, 0, 0b0100_0000).await?;
        Ok(self)
    }

    /// temp_conversion_rate get the current conversion rate in Hertz.
    pub async fn temp_conversion_rate(&mut self) -> Result<ConversionRate, I::Error> {
        trace!("temp_conversion_rate");
        let rate: ConversionRate = match self.read_reg(Register::ConversionRate).await? {
            0 => ConversionRate::Rate1_16Hz,
            1 => ConversionRate::Rate1_8Hz,
            2 => ConversionRate::Rate1_4Hz,
            3 => ConversionRate::Rate1_2Hz,
            4 => ConversionRate::Rate1Hz,
            5 => ConversionRate::Rate2Hz,
            6 => ConversionRate::Rate4Hz,
            7 => ConversionRate::Rate8Hz,
            8 => ConversionRate::Rate16Hz,
            9..=15 => ConversionRate::Rate32Hz,
            _ => return Err(Error::InvalidValue),
        };
        Ok(rate)
    }

    /// set_ideality_factor set the ideality factor applied to the External Diode, to match a
    /// diode (such as a processor thermal diode) whose ideality factor differs from 1.008.
    /// The 'setting' must be between 0x08 (0.9949) and 0x37 (1.0566), the default 0x12 being
    /// 1.0080. See the Ideality Factor Look-Up Table in the datasheet for every setting.
    pub async fn set_ideality_factor(
        &mut self,
        setting: BoundedU8<0x08, 0x37>,
    ) -> Result<&mut Self, I::Error> {
        trace!("set_ideality_factor");
        // ExternalIdealityFactor[5:0] IDCF : ideality factor applied to the External Diode.
        self.write_reg(Register::ExternalIdealityFactor, setting.get())
            .await?;
        Ok(self)
    }

    /// set_beta_compensation set the Beta Compensation of the External Diode channel.
    pub async fn set_beta_compensation(
        &mut self,
        beta: BetaCompensation,
    ) -> Result<&mut Self, I::Error> {
        trace!("set_beta_compensation");
        // BetaCompensation[3] ENABLE : set for automatic detection, cleared to use BETA[2:0].
        // BetaCompensation[2:0] BETA : fixed Beta Compensation factor, 7 disables it.
        self.write_reg(Register::BetaCompensation, beta as u8)
            .await?;
        Ok(self)
    }

    /// temp_internal read the internal temperature value in degree Celsius.
    pub async fn temp_internal(&mut self) -> Result<i8, I::Error> {
        trace!("temp_internal");
        Ok(self.read_reg(Register::InternalTemperature).await? as i8)
    }

    /// temp_external read the external temperature value in degree Celsius.
    pub async fn temp_external(&mut self) -> Result<i8, I::Error> {
        trace!("temp_external");
        Ok(self.read_reg(Register::ExternalTemperatureMSB).await? as i8)
    }

    /// temp_external_precise read the external temperature value in degree Celsius.
    pub async fn temp_external_precise(&mut self) -> Result<f32, I::Error> {
        trace!("temp_external_precise");
        // ExternalTemperatureMSB must be read first: reading it latches ExternalTemperatureLSB into
        // a shadow register, and reading ExternalTemperatureLSB returns that latched value (Data
        // Read Interlock).
        let msb = self.read_reg(Register::ExternalTemperatureMSB).await?;
        let lsb = self.read_reg(Register::ExternalTemperatureLSB).await?;
        let raw: i16 = (((msb as u16) << 8) + lsb as u16) as i16;
        let ret: f32 = (raw >> 5) as f32 * 0.125;
        Ok(ret)
    }

    /// monitor_temp_internal_high start monitoring the internal temperature and will create
    /// an alert when the temperature exceeds the limit.
    /// The temp_int_high will be true in Status until the internal temperature drops below the high limit.
    pub async fn monitor_temp_internal_high(&mut self, limit: i8) -> Result<&mut Self, I::Error> {
        trace!("monitor_temp_internal_high");
        // If the measured temperature for the internal diode exceeds the Internal Temperature limit,
        // then the INT_HIGH bit is set in the Status Register. It remains set until the internal
        // temperature drops below the high limit.
        self.write_reg(Register::InternalTempLimit, limit as u8)
            .await?;
        // Clear AlertMask[6] INT_MSK : The Internal Diode will generate an interrupt if measured temperature
        // exceeds the Internal Diode high limit.
        self.update_reg(Register::AlertMask, 0, 0b0100_0000).await?;
        Ok(self)
    }

    /// monitor_temp_external_critical start monitoring the external temperature and will create
    /// an alert when the temperature exceeds the critical limit.
    /// The temp_ext_critical will be true in Status until the external temperature drops below the critical
    /// limit minus critical hysteresis.
    pub async fn monitor_temp_external_critical(
        &mut self,
        limit: i8,
        hysteresis: u8,
    ) -> Result<&mut Self, I::Error> {
        trace!("monitor_temp_external_critical");
        // If the external diode exceeds the TCRIT Temp limit (even if it does not exceeds the External Diode
        // Temperature Limit), the TCRIT bit is set in the Status Register. It remains set until the external
        // temperature drops below the Critical Limit minus the Critical Hysteresis.
        self.write_reg(Register::ExternalTempCriticalLimit, limit as u8)
            .await?;
        self.write_reg(Register::ExternalTempCriticalHysteresis, hysteresis)
            .await?;
        // Clear AlertMask[1] TCRIT_MSK : An interrupt will be generated if the External Diode temperature
        // exceeds the TCRIT limit.
        self.update_reg(Register::AlertMask, 0, 0b0000_0010).await?;
        Ok(self)
    }

    /// monitor_temp_external_high start monitoring the external temperature and will create
    /// an alert when the temperature exceeds the high limit.
    /// The temp_ext_high will be true in Status until the external temperature drops below the high limit.
    pub async fn monitor_temp_external_high(&mut self, limit: i8) -> Result<&mut Self, I::Error> {
        trace!("monitor_temp_external_high");
        // If the measured temperature for the external diode exceeds the External Temperature High limit,
        // then the EXT_HIGH bit is set in the Status Register. It remains set until the external
        // temperature drops below the high limit.
        self.write_reg(Register::ExternalTempLimitHigh, limit as u8)
            .await?;
        // Clear AlertMask[4] HIGH_MSK : The External Diode will generate an interrupt if measured temperature
        // exceeds the External Diode high limit.
        self.update_reg(Register::AlertMask, 0, 0b0001_0000).await?;
        Ok(self)
    }

    /// monitor_temp_external_low start monitoring the external temperature and will create
    /// an alert when the temperature drops below the low limit.
    /// The temp_ext_low will be true in Status until the external temperature exceeds the low limit.
    pub async fn monitor_temp_external_low(&mut self, limit: i8) -> Result<&mut Self, I::Error> {
        trace!("monitor_temp_external_low");
        // If the measured temperature for the external diode drops below the External Temperature Low limit,
        // then the EXT_LOW bit is set in the Status Register. It remains set until the external
        // temperature exceeds the low limit.
        self.write_reg(Register::ExternalTempLimitLow, limit as u8)
            .await?;
        // Clear AlertMask[3] LOW_MSK : The External Diode will generate an interrupt if measured temperature
        // drops below the External Diode low limit.
        self.update_reg(Register::AlertMask, 0, 0b0000_1000).await?;
        Ok(self)
    }

    /// enable_alert_output configure ALERT#/TACH pin as open drain active low ALERT# interrupt.
    /// This may require an external pull-up resistor to set the proper signaling levels.
    pub async fn enable_alert_output(&mut self) -> Result<&mut Self, I::Error> {
        trace!("enable_alert_output");
        // Clear Configuration[2] ALT_TCH : The ALERT#/TACH pin will function as open drain,
        // active low interrupt.
        // Clear Configuration[7] MASK : The ALERT#/TACH pin will be asserted if any bit is set in the
        // Status Register. Once the pin is asserted, it remains asserted.
        self.update_reg(Register::Configuration, 0, 0b1000_0100)
            .await?;
        Ok(self)
    }

    /// enable_tach_input configure ALERT#/TACH pin as high impedance TACH input.
    pub async fn enable_tach_input(&mut self) -> Result<&mut Self, I::Error> {
        trace!("enable_tach_input");
        // Set Configuration[2] ALT_TCH : The ALERT#/TACH pin will function as high impedance TACH input.
        self.update_reg(Register::Configuration, 0b0000_0100, 0)
            .await?;
        Ok(self)
    }

    /// set_fan_pwm set FAN in PWM mode at the given frequency, with the maximum PWM resolution.
    /// The 'frequency' must be between 23 Hz and 5806 Hz.
    /// The actual frequency is 360 kHz / (62 * PWM_D), with PWM_D the rounded divider (1 to 252),
    /// so the available frequencies get coarser at the top of the range (5806 Hz, 2903 Hz, 1935 Hz, ...).
    /// In PWM mode, a fan step of 62 or 63 gives a 100% duty cycle.
    pub async fn set_fan_pwm(
        &mut self,
        frequency: HertzU32,
        inverted: bool,
    ) -> Result<&mut Self, I::Error> {
        trace!("set_fan_pwm");
        // The PWM frequency when the PWMFrequencyDivide Register is used is shown in equation :
        // FREQ = 360k / (2 * PWM_F * PWM_D)
        const BASE_CLOCK: u32 = 360_000;
        // Maximum resolution is achieved by setting the PWMFrequency Register to 1Fh.
        const PWM_F: u32 = 0x1F;
        // set_fan_pwm_advanced takes PWM_F as BoundedU8::MAX.
        const _: () = ::core::assert!(PWM_F == BoundedU8::<1, 31>::MAX_VALUE as u32);
        let freq = frequency.to_raw();
        // PWM_D[7:0] must stay between 1 and 255.
        if !(23..=BASE_CLOCK / (2 * PWM_F)).contains(&freq) {
            error!("Invalid PWM Frequency.");
            return Err(Error::InvalidValue);
        }
        // PWM_D = 360k / (2 * PWM_F * FREQ), rounded to the nearest integer.
        let pwm_d = (BASE_CLOCK + PWM_F * freq) / (2 * PWM_F * freq);
        let pwm_d = BoundedU8::new(pwm_d as u8).ok_or(Error::InvalidValue)?;
        self.set_fan_pwm_advanced(PwmClock::Divided(pwm_d), BoundedU8::MAX, inverted)
            .await
    }

    /// set_fan_pwm_advanced set FAN in PWM mode with the given base clock and PWM_F setting
    /// (see "Advanced PWM Options" in the datasheet).
    /// The PWM frequency is the base clock / (2 * 'pwm_f'), and a fan step of 2 * 'pwm_f' or more
    /// gives a 100% duty cycle, so a 'pwm_f' below 31 trades PWM resolution for frequency.
    /// For example, the power-on setting (360 kHz base clock and a 'pwm_f' of 0x17) gives about
    /// 7.8 kHz, with a 100% duty cycle from fan step 46.
    pub async fn set_fan_pwm_advanced(
        &mut self,
        clock: PwmClock,
        pwm_f: BoundedU8<1, 31>,
        inverted: bool,
    ) -> Result<&mut Self, I::Error> {
        trace!("set_fan_pwm_advanced");
        // PWMFrequency[4:0] PWM_F : the number of base clocks in half a PWM period. It determines
        // the final PWM frequency and "effective resolution" of the PWM driver.
        self.write_reg(Register::PWMFrequency, pwm_f.get()).await?;
        let (clk_set, clk_clear) = match clock {
            // Clear FanConfig[3] CLK_SEL and FanConfig[2] CLK_OVR : The base clock that is used to
            // determine the PWM frequency is 360kHz.
            PwmClock::Base360kHz => (0, 0b0000_1100),
            // Set FanConfig[3] CLK_SEL and clear FanConfig[2] CLK_OVR : The base clock that is used
            // to determine the PWM frequency is 1.4kHz.
            PwmClock::Base1400Hz => (0b0000_1000, 0b0000_0100),
            PwmClock::Divided(pwm_d) => {
                // When the CLK_OVR bit is set to a logic '1', the PWMFrequencyDivide Register is
                // used in conjunction with the PWMFrequency Register to determine the final PWM
                // frequency that the load will see.
                self.write_reg(Register::PWMFrequencyDivide, pwm_d.get())
                    .await?;
                // Set FanConfig[2] CLK_OVR : The base clock that is used to determine the PWM
                // frequency is set by the Frequency Divide Register.
                (0b0000_0100, 0)
            }
        };
        if inverted {
            // Set FanConfig[4] POLARITY : The polarity of the Fan output driver is inverted.
            // A 0x00 setting will correspond to a 100% duty cycle or maximum DAC output voltage.
            self.update_reg(Register::FanConfig, clk_set | 0b0001_0000, clk_clear)
                .await?;
        } else {
            // Clear FanConfig[4] POLARITY : The polarity of the Fan output driver is non-inverted.
            // A 0x00 setting will correspond to a 0% duty cycle or minimum DAC output voltage.
            self.update_reg(Register::FanConfig, clk_set, clk_clear | 0b0001_0000)
                .await?;
        }
        // Clear Configuration[4] DAC : PWM output enabled at FAN pin.
        self.update_reg(Register::Configuration, 0, 0b0001_0000)
            .await?;
        Ok(self)
    }

    /// set_fan_dac set FAN in DAC mode.
    pub async fn set_fan_dac(&mut self, inverted: bool) -> Result<&mut Self, I::Error> {
        trace!("set_fan_dac");
        // Set Configuration[4] DAC : DAC output enabled at FAN pin.
        self.update_reg(Register::Configuration, 0b0001_0000, 0)
            .await?;
        if inverted {
            // Set FanConfig[4] POLARITY : The polarity of the Fan output driver is inverted.
            // A 0x00 setting will correspond to a 100% duty cycle or maximum DAC output voltage.
            self.update_reg(Register::FanConfig, 0b0001_0000, 0).await?;
        } else {
            // Clear FanConfig[4] POLARITY : The polarity of the Fan output driver is non-inverted.
            // A 0x00 setting will correspond to a 0% duty cycle or minimum DAC output voltage.
            self.update_reg(Register::FanConfig, 0, 0b0001_0000).await?;
        }
        Ok(self)
    }

    /// set_fan_power set the FAN power (for both modes PWM/DAC).
    /// The 'step' must be between 0 and 63.
    /// If Look-up Table was enabled, it will be disabled and the fixed power value will be used.
    pub async fn set_fan_power(&mut self, step: BoundedU8<0, 63>) -> Result<&mut Self, I::Error> {
        trace!("set_fan_power");
        // FanConfig is only read (and written back) while the Look-Up Table state is unknown or
        // active, not on every call, so a corrupted read can't be written back from a fan loop.
        if self.lut_active != Some(false) {
            let fan_config: u8 = self.read_reg(Register::FanConfig).await?;
            // FanConfig[5] PROG == 0 :
            // the FanSetting Register and Fan Control Look-Up Table Registers are read-only.
            if fan_config & 0x20 == 0x00 {
                // Set FanConfig[5] PROG : the FanSetting Register and Fan Control Look-Up
                // Table Registers can be written and the Fan Control Look-Up Table Registers
                // will not be used.
                self.write_reg(Register::FanConfig, fan_config | 0x20)
                    .await?;
            }
            self.lut_active = Some(false);
        }
        // The FanSetting Register drives the fan driver when the Fan Control Look-Up
        // Table is not used.
        // Any data written to the FanSetting register is applied immediately to the
        // fan driver (PWM or DAC).
        self.write_reg(Register::FanSetting, step.get()).await?;
        Ok(self)
    }

    /// set_fan_lut set the FAN according to a Look-up Table with hysteresis.
    /// The 'lut' must be 8 or less levels and be ordered with lower temperature value first.
    /// Each level temperature must be between 0 and 127 degrees Celsius.
    /// Each level step must be between 0 and 63.
    pub async fn set_fan_lut(
        &mut self,
        lut: Vec<Level, 8>,
        hysteresis: BoundedU8<0, 31>,
    ) -> Result<&mut Self, I::Error> {
        trace!("set_fan_lut");
        if lut.is_empty() {
            return Err(Error::InvalidSize);
        }
        // Unknown until the Look-Up Table is fully set, in case of an error midway.
        self.lut_active = None;
        let fan_config: u8 = self.read_reg(Register::FanConfig).await?;
        // FanConfig[5] PROG == 0 :
        // the FanSetting Register and Fan Control Look-Up Table Registers are read-only
        // and the Fan Control Look-Up Table Registers will be used.
        if fan_config & 0x20 == 0x00 {
            // Set FanConfig[5] PROG : the FanSetting Register and Fan Control Look-Up
            // Table Registers can be written and the Fan Control Look-Up Table Registers
            // will not be used.
            self.write_reg(Register::FanConfig, fan_config | 0x20)
                .await?;
        }
        let mut last_temp = <BoundedU8<0, 127>>::new(0).unwrap();
        for (index, level) in (0_u8..).zip(lut.iter()) {
            if level.temp <= last_temp {
                return Err(Error::InvalidSorting);
            }
            last_temp = level.temp;
            self.write_reg(
                (Register::FanControlLUTT1 as u8) + 2 * index,
                level.temp.get(),
            )
            .await?;
            self.write_reg(
                (Register::FanControlLUTS1 as u8) + 2 * index,
                level.step.get(),
            )
            .await?;
        }
        // FanControlLUTHysteresis determines the amount of hysteresis applied to the temperature
        // inputs of the fan control Fan Control Look-Up Table.
        self.write_reg(Register::FanControlLUTHysteresis, hysteresis.get())
            .await?;
        // Clear FanConfig[5] PROG : the FanSetting Register and Fan Control Look-Up Table
        // Registers are read-only and the Fan Control Look-Up Table Registers will be used.
        self.write_reg(Register::FanConfig, fan_config & !0x20)
            .await?;
        self.lut_active = Some(true);
        Ok(self)
    }

    /// fan_rpm gives the Fan speed in RPM.
    /// A TACH count of 0 or 0xFFFF gives 0 RPM, and speeds above 65535 RPM are clamped.
    pub async fn fan_rpm(&mut self) -> Result<u16, I::Error> {
        trace!("fan_rpm");
        // TachLSB must be read first: reading it latches TachMSB into a shadow register, and
        // reading TachMSB returns that latched value (Data Read Interlock).
        let lsb = self.read_reg(Register::TachLSB).await?;
        let msb = self.read_reg(Register::TachMSB).await?;
        // A count of 0 can't come from a spinning fan and would divide by zero, and counts below 83
        // give more than u16::MAX RPM.
        Ok(match u16::from_be_bytes([msb, lsb]) {
            0 | 0xFFFF => 0,
            count => u16::try_from(5_400_000 / u32::from(count)).unwrap_or(u16::MAX),
        })
    }

    /// read_reg read a register value.
    async fn read_reg<R: Into<u8>>(&mut self, reg: R) -> Result<u8, I::Error> {
        trace!("read_reg");
        let mut buf = [0x00];
        let reg = reg.into();
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .await
            .map_err(Error::I2c)?;
        debug!("R @0x{:x}={:x}", reg, buf[0]);
        Ok(buf[0])
    }

    /// write_reg blindly write a single register with a fixed value.
    async fn write_reg<R: Into<u8>>(&mut self, reg: R, value: u8) -> Result<(), I::Error> {
        trace!("write_reg");
        let reg = reg.into();
        debug!("W @0x{:x}={:x}", reg, value);
        self.i2c
            .write(self.address, &[reg, value])
            .await
            .map_err(Error::I2c)
    }

    /// update_reg first read the register value, apply a set mask, then a clear mask, and write the new value
    /// only if different from the initial value.
    async fn update_reg<R: Into<u8> + Clone>(
        &mut self,
        reg: R,
        mask_set: u8,
        mask_clear: u8,
    ) -> Result<(), I::Error> {
        trace!("update_reg");
        let current = self.read_reg(reg.clone()).await?;
        let updated = (current | mask_set) & !mask_clear;
        if current != updated {
            self.write_reg(reg, updated).await?;
        }
        Ok(())
    }

    /// Return the underlying I2C device
    pub fn release(self) -> I {
        self.i2c
    }

    /// Destroys this driver and releases the I2C bus `I`.
    pub fn destroy(self) -> Self {
        self
    }
}

#[cfg(test)]
mod test {
    // extern crate alloc;
    extern crate std;

    use super::*;
    use embedded_hal_mock::eh1::i2c;
    use std::vec;

    #[test]
    fn new_emc2101() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
        ];
        let mock = i2c::Mock::new(&expectations);
        let emc2101 = EMC2101::new(mock).unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn set_fan_lut_enables_lut_control() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            // PROG is clear, so it gets set to unlock the LUT registers.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8],
                vec![0b0000_1000],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0010_1000],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::FanControlLUTT1 as u8, 30]),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::FanControlLUTS1 as u8, 20]),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanControlLUTT1 as u8 + 2, 60],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanControlLUTS1 as u8 + 2, 63],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanControlLUTHysteresis as u8, 4],
            ),
            // PROG must be cleared at the end so the LUT drives the fan (issue #4).
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0000_1000],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        let lut = Vec::<Level, 8>::from_slice(&[
            Level {
                temp: BoundedU8::new(30).unwrap(),
                step: BoundedU8::new(20).unwrap(),
            },
            Level {
                temp: BoundedU8::new(60).unwrap(),
                step: BoundedU8::new(63).unwrap(),
            },
        ])
        .unwrap();
        emc2101
            .set_fan_lut(lut, BoundedU8::new(4).unwrap())
            .unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn monitor_temp_internal_high_unmasks_alert() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::InternalTempLimit as u8, 70]),
            // AlertMask[6] INT_MSK must be cleared.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::AlertMask as u8],
                vec![0xFF],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::AlertMask as u8, 0b1011_1111],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        emc2101.monitor_temp_internal_high(70).unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn monitor_temp_external_critical_unmasks_tcrit_alert() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::ExternalTempCriticalLimit as u8, 85],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::ExternalTempCriticalHysteresis as u8, 10],
            ),
            // AlertMask[1] TCRIT_MSK must be cleared, not AlertMask[4] HIGH_MSK.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::AlertMask as u8],
                vec![0xFF],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::AlertMask as u8, 0b1111_1101],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        emc2101.monitor_temp_external_critical(85, 10).unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn configure_adc_sets_and_clears_filter_bits() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![
                    Register::ConversionRate as u8,
                    ConversionRate::Rate1Hz as u8,
                ],
            ),
            // Level2 (FILTER = 11b) with ALERT_COMP set, going to Level1 (FILTER = 01b):
            // FILTER[1] is cleared, FILTER[0] and ALERT_COMP are kept.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::AveragingFilter as u8],
                vec![0b0000_0111],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::AveragingFilter as u8, 0b0000_0011],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        emc2101
            .configure_adc(ConversionRate::Rate1Hz, FilterLevel::Level1)
            .unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn set_fan_pwm_uses_frequency_divide_register() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            // 25 Hz : PWM_D = 360k / (2 * 31 * 25) = 232.26 -> 232
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::PWMFrequency as u8, 0x1F]),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::PWMFrequencyDivide as u8, 232],
            ),
            // PROG and POLARITY set : CLK_OVR gets set, POLARITY cleared.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8],
                vec![0b0011_0000],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0010_0100],
            ),
            // DAC mode was enabled : it gets cleared.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8],
                vec![0b0001_0000],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::Configuration as u8, 0]),
            // 100 Hz inverted : PWM_D = 360k / (2 * 31 * 100) = 58.06 -> 58
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::PWMFrequency as u8, 0x1F]),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::PWMFrequencyDivide as u8, 58],
            ),
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8],
                vec![0b0010_0100],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0011_0100],
            ),
            // Already in PWM mode : no write.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8],
                vec![0],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        emc2101.set_fan_pwm(HertzU32::from_raw(25), false).unwrap();
        emc2101.set_fan_pwm(HertzU32::from_raw(100), true).unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn set_fan_pwm_rejects_out_of_range_frequency() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        for freq in [0, 22, 5_807, 25_000, 360_000] {
            assert!(matches!(
                emc2101.set_fan_pwm(HertzU32::from_raw(freq), false),
                Err(Error::InvalidValue)
            ));
        }

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn fan_rpm_reads_tach_low_byte_first() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            // Reading TachLSB latches TachMSB, so TachLSB must come first.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::TachLSB as u8],
                vec![0x38],
            ),
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::TachMSB as u8],
                vec![0x04],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        // 5_400_000 / 0x0438 (1080 counts) = 5000 RPM.
        assert_eq!(emc2101.fan_rpm().unwrap(), 5000);

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn temp_external_precise_reads_high_byte_first() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            // Reading ExternalTemperatureMSB latches ExternalTemperatureLSB, so the MSB must come
            // first.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ExternalTemperatureMSB as u8],
                vec![0x19],
            ),
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ExternalTemperatureLSB as u8],
                vec![0x60],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        // 0x1960 >> 5 = 203 eighths of a degree.
        assert_eq!(emc2101.temp_external_precise().unwrap(), 25.375);

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn set_ideality_factor_and_beta_compensation() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::ExternalIdealityFactor as u8, 0x24],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::BetaCompensation as u8, 0x00],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::BetaCompensation as u8, 0x07],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::BetaCompensation as u8, 0x08],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        emc2101
            .set_ideality_factor(BoundedU8::new(0x24).unwrap())
            .unwrap()
            .set_beta_compensation(BetaCompensation::Min0_11)
            .unwrap()
            .set_beta_compensation(BetaCompensation::Disabled)
            .unwrap()
            .set_beta_compensation(BetaCompensation::Auto)
            .unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn beta_compensation_from_raw_value() {
        assert_eq!(BetaCompensation::try_from(0), Ok(BetaCompensation::Min0_11));
        assert_eq!(BetaCompensation::try_from(6), Ok(BetaCompensation::Min2_33));
        assert_eq!(
            BetaCompensation::try_from(7),
            Ok(BetaCompensation::Disabled)
        );
        assert_eq!(BetaCompensation::try_from(8), Ok(BetaCompensation::Auto));
        // ENABLE set : the BETA bits are ignored.
        assert_eq!(BetaCompensation::try_from(0x0F), Ok(BetaCompensation::Auto));
        assert_eq!(BetaCompensation::try_from(0x10), Err(Error::InvalidValue));
    }

    #[test]
    fn set_fan_power_reads_fan_config_only_when_lut_state_unknown() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            // First call : LUT state unknown, a previous firmware left PROG clear.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8],
                vec![0b0000_1000],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0010_1000],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::FanSetting as u8, 10]),
            // LUT known inactive : FanConfig is not read again.
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::FanSetting as u8, 20]),
            // set_fan_lut : PROG already set, then cleared at the end.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8],
                vec![0b0010_1000],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::FanControlLUTT1 as u8, 30]),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::FanControlLUTS1 as u8, 20]),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanControlLUTHysteresis as u8, 4],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0000_1000],
            ),
            // LUT active : PROG gets set again, once.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8],
                vec![0b0000_1000],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0010_1000],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::FanSetting as u8, 30]),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::FanSetting as u8, 40]),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        let lut = Vec::<Level, 8>::from_slice(&[Level {
            temp: BoundedU8::new(30).unwrap(),
            step: BoundedU8::new(20).unwrap(),
        }])
        .unwrap();
        emc2101
            .set_fan_power(BoundedU8::new(10).unwrap())
            .unwrap()
            .set_fan_power(BoundedU8::new(20).unwrap())
            .unwrap()
            .set_fan_lut(lut, BoundedU8::new(4).unwrap())
            .unwrap()
            .set_fan_power(BoundedU8::new(30).unwrap())
            .unwrap()
            .set_fan_power(BoundedU8::new(40).unwrap())
            .unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn set_standby_and_one_shot() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            // ALT_TCH set : STANDBY gets set, ALT_TCH kept.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8],
                vec![0b0000_0100],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8, 0b0100_0100],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::OneShot as u8, 0]),
            // STANDBY gets cleared, ALT_TCH kept.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8],
                vec![0b0100_0100],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8, 0b0000_0100],
            ),
            // Already operational : no write.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8],
                vec![0b0000_0100],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        emc2101
            .set_standby(true)
            .unwrap()
            .one_shot()
            .unwrap()
            .set_standby(false)
            .unwrap()
            .set_standby(false)
            .unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn set_fan_pwm_advanced_selects_base_clock() {
        let expectations = [
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
            // 360 kHz, PWM_F 0x17 : CLK_SEL, CLK_OVR and POLARITY cleared, the divider untouched.
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::PWMFrequency as u8, 0x17]),
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8],
                vec![0b0011_1100],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0010_0000],
            ),
            // DAC mode was enabled : it gets cleared.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8],
                vec![0b0001_0000],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::Configuration as u8, 0]),
            // 1.4 kHz inverted, PWM_F 0x1F : CLK_SEL and POLARITY set, CLK_OVR cleared.
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::PWMFrequency as u8, 0x1F]),
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8],
                vec![0b0010_0100],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0011_1000],
            ),
            // Already in PWM mode : no write.
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8],
                vec![0],
            ),
            // 360 kHz / 10, PWM_F 0x08 : CLK_OVR set, POLARITY cleared, CLK_SEL left as is.
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::PWMFrequency as u8, 0x08]),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::PWMFrequencyDivide as u8, 10],
            ),
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8],
                vec![0b0011_1000],
            ),
            i2c::Transaction::write(
                DEFAULT_ADDRESS,
                vec![Register::FanConfig as u8, 0b0010_1100],
            ),
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::Configuration as u8],
                vec![0],
            ),
        ];
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        emc2101
            .set_fan_pwm_advanced(PwmClock::Base360kHz, BoundedU8::new(0x17).unwrap(), false)
            .unwrap()
            .set_fan_pwm_advanced(PwmClock::Base1400Hz, BoundedU8::new(0x1F).unwrap(), true)
            .unwrap()
            .set_fan_pwm_advanced(
                PwmClock::Divided(BoundedU8::new(10).unwrap()),
                BoundedU8::new(0x08).unwrap(),
                false,
            )
            .unwrap();

        let mut mock = emc2101.release();
        mock.done();
    }

    #[test]
    fn fan_rpm_handles_out_of_range_counts() {
        // (TACH count, expected RPM)
        let cases: [(u16, u16); 5] = [
            (0x0000, 0),    // would divide by zero (issue #5)
            (0xFFFF, 0),    // fan stopped
            (1, u16::MAX),  // 5_400_000 RPM, clamped
            (82, u16::MAX), // 65853 RPM, clamped
            (83, 65060),    // largest speed that fits in u16
        ];
        let mut expectations = vec![
            i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::ProductID as u8],
                vec![EMC2101_PRODUCT_ID],
            ),
            i2c::Transaction::write(DEFAULT_ADDRESS, vec![Register::AlertMask as u8, 0xFF]),
        ];
        for (count, _) in cases {
            let [msb, lsb] = count.to_be_bytes();
            expectations.push(i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::TachLSB as u8],
                vec![lsb],
            ));
            expectations.push(i2c::Transaction::write_read(
                DEFAULT_ADDRESS,
                vec![Register::TachMSB as u8],
                vec![msb],
            ));
        }
        let mock = i2c::Mock::new(&expectations);
        let mut emc2101 = EMC2101::new(mock).unwrap();

        for (count, rpm) in cases {
            assert_eq!(emc2101.fan_rpm().unwrap(), rpm, "TACH count {}", count);
        }

        let mut mock = emc2101.release();
        mock.done();
    }
}
