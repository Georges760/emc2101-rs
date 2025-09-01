pub type Result<T, E> = core::result::Result<T, Error<E>>;

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// I2C bus error.
    I2c(E),
    /// The device Product ID is not supported.
    InvalidID,
    /// The given Value is not valid.
    InvalidValue,
    /// The given array as an invalid size.
    InvalidSize,
    /// The given array is not sorted.
    InvalidSorting,
    /// Errors such as overflowing the stack.
    Internal,
}

#[rustversion::since(1.81)]
impl<E: core::fmt::Debug> core::error::Error for Error<E> {}

#[rustversion::since(1.81)]
impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{self:?}")
    }
}
