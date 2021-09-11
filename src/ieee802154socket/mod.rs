#![deny(unused_must_use)]
#![no_main]
#![no_std]

use hal::ieee802154::{self};

use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
use smoltcp::time::Instant;
use smoltcp::Result;

/// A socket that captures or transmits the complete frame.
//#[derive(Debug)]
pub struct IEEE802154Socket<'a> {
    lower: &'a mut ieee802154::Radio<'a>,
    mtu: usize,
    receiving: bool,
    rx_buffer: ieee802154::Packet,
}


impl<'a> IEEE802154Socket<'a> {
    /// Creates a raw socket, bound to the interface called `name`.
    ///
    /// This requires superuser privileges or a corresponding capability bit
    /// set on the executable.
    pub fn new(mut radio: ieee802154::Radio<'a>) -> Result<IEEE802154Socket> {
        Ok(IEEE802154Socket {
            lower: &mut radio,
            mtu: ieee802154::Packet::CAPACITY as usize,
            receiving: false,
            rx_buffer: ieee802154::Packet::new(),
        })
    }
}

impl<'a> Device<'a> for IEEE802154Socket<'a> {
    type RxToken = RxToken;
    type TxToken = TxToken<'a>;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit =  self.mtu;
        caps.medium = Medium::Ieee802154;
        caps.max_burst_size = Some(1);
        caps
    }

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let lower = &mut*self.lower;
        
        if !self.receiving {
            lower.recv_async_start(&mut self.rx_buffer);
            self.receiving = true;
        }

        if lower.recv_async_poll() {
            let res = lower.recv_async_sync();
            self.receiving = false;

            let mut buffer = vec![0; self.rx_buffer.len() as usize];
            buffer.copy_from_slice(&*self.rx_buffer);


            let rx = RxToken { buffer };
            let tx = TxToken {
                lower: &mut self.lower,
            };
            Some((rx, tx))
        } else {
            None
        }
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(TxToken {
            lower: &mut self.lower,
        })
    }

}

#[doc(hidden)]
pub struct RxToken {
    buffer: Vec<u8>,
}

impl phy::RxToken for RxToken {
    fn consume<R, F>(mut self, _timestamp: Instant, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        f(&mut self.buffer[..])
    }
}

#[doc(hidden)]
pub struct TxToken<'a> {
    lower: &'a mut ieee802154::Radio<'a>,
}

impl<'a> phy::TxToken for TxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        let mut lower = self.lower;
        if self.receiving {
            lower.cancel_recv();
            self.receiving = false;
        }
        let mut buffer = ieee802154::Packet::new();
        let result = f(&mut buffer);
        lower.send(&mut buffer);
        result
    }
}