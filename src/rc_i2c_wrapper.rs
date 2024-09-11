/// embedded-hal-bus is actually bad, so I modified the code.
/// This code is still bad, because it relies on runtime evaluation of ownership.
///
/// 'guess I'll have to cry about it.
use core::cell::RefCell;

use embassy_sync::mutex::Mutex;
// use portable_atomic::*;
// use async_lock::Mutex;

pub struct RefCellI2c<'a, T> {
    bus: &'a RefCell<T>,
}

impl<'a, T> RefCellI2c<'a, T> {
    /// Create a new `RefCellDevice`.
    #[inline]
    pub fn new(bus: &'a RefCell<T>) -> Self {
        Self { bus }
    }
}

impl<'a, T> embedded_hal::i2c::ErrorType for RefCellI2c<'a, T>
where
    T: embedded_hal::i2c::I2c,
{
    type Error = T::Error;
}

impl<'a, T> embedded_hal::i2c::I2c for RefCellI2c<'a, T>
where
    T: embedded_hal::i2c::I2c,
{
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let bus = &mut *self.bus.borrow_mut();
        bus.transaction(address, operations)
    }
}

impl<'a, T> embedded_hal_async::i2c::I2c for RefCellI2c<'a, T>
where
    T: embedded_hal_async::i2c::I2c + embedded_hal::i2c::I2c,
{
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let bus = &mut *self.bus.borrow_mut();

        embedded_hal_async::i2c::I2c::transaction(bus, address, operations).await
    }
}

pub struct MutexI2cBus<M: embassy_sync::blocking_mutex::raw::RawMutex, T> {
    bus: embassy_sync::mutex::Mutex<M, T>,
}

impl<M, T> MutexI2cBus<M, T>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex,
{
    /// Create a new `RefCellDevice`.
    #[inline]
    pub fn new(bus: Mutex<M, T>) -> Self {
        Self { bus }
    }
}

impl<M, T> embedded_hal::i2c::ErrorType for MutexI2cBus<M, T>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex,
    T: embedded_hal::i2c::I2c,
{
    type Error = T::Error;
}

impl<M, T> embedded_hal_async::i2c::I2c for MutexI2cBus<M, T>
where
    M: embassy_sync::blocking_mutex::raw::RawMutex,
    T: embedded_hal_async::i2c::I2c + embedded_hal::i2c::I2c,
{
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let bus = &mut *self.bus.lock().await;
        embedded_hal_async::i2c::I2c::transaction(bus, address, operations).await
    }
}
