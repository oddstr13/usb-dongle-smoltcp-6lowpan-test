#![deny(unused_must_use)]
#![no_std]

use hal::ieee802154::{self};

use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
use smoltcp::time::Instant;
use smoltcp::Result;

/// A socket that captures or transmits the complete frame.
#[derive(Debug)]
pub struct IEEE802154Socket {
    lower: ieee802154::Radio<'static>,
    mtu: usize,
}


impl IEEE802154Socket {
    pub fn new(radio: ieee802154::Radio<'static>) -> Result<IEEE802154Socket> {
        Ok(IEEE802154Socket {
            lower: radio,
            mtu: ieee802154::Packet::CAPACITY as usize,
        })
    }
}

impl<'a> Device<'a> for IEEE802154Socket {
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

        match self.lower.recv_async() {
            Ok(packet) => {
                log::debug!("received {} bytes", packet.len());
                let rx = RxToken { buffer:packet };
                let tx = TxToken {
                    lower: &mut self.lower,
                };
                Some((rx, tx))
            },
            Err(nb::Error::WouldBlock) => None,
            Err(nb::Error::Other(ieee802154::Error::AsyncCrc(_packet, _crc))) => None,
            Err(_) => None,
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
    buffer: ieee802154::Packet,
}

impl phy::RxToken for RxToken {
    fn consume<R, F>(mut self, _timestamp: Instant, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        log::debug!("RxToken consume");
        f(&mut self.buffer)
    }
}

#[doc(hidden)]
pub struct TxToken<'a> {
    lower: &'a mut ieee802154::Radio<'static>,
}

impl<'a> phy::TxToken for TxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        log::debug!("TxToken consume {}", len);
        let mut buffer = ieee802154::Packet::new();
        buffer.set_len(len as u8);

        let result = f(&mut *buffer);

        self.lower.send(&mut buffer);
        result
    }
}