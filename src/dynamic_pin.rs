use alloc::boxed::Box;
use core::ops::Deref;
use imxrt_hal::gpio::Input;
use imxrt_hal::gpio::GPIO;
use imxrt_hal::iomuxc::gpio::Pin;

pub trait DynamicPinTrait {
    fn is_set(&self) -> bool;
}

//

pub struct DynamicPinCore<P> {
    base: GPIO<P, Input>,
}

impl<P> DynamicPinCore<P> {
    pub fn new(base: GPIO<P, Input>) -> Self {
        DynamicPinCore { base }
    }
}

impl<P: Pin> DynamicPinTrait for DynamicPinCore<P> {
    fn is_set(&self) -> bool {
        self.base.is_set()
    }
}

pub struct DynamicPin<'a> {
    base: Box<dyn DynamicPinTrait + 'a>,
}

impl<'a> DynamicPin<'a> {
    pub fn new<P: Pin + 'a>(p0: GPIO<P, Input>) -> DynamicPin<'a> {
        DynamicPin {
            base: Box::new(DynamicPinCore::new(p0)),
        }
    }
}

impl<'a> Deref for DynamicPin<'a> {
    type Target = dyn DynamicPinTrait + 'a;

    fn deref(&self) -> &Self::Target {
        &*self.base
    }
}
