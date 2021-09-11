#![deny(unused_must_use)]
#![no_main]
#![no_std]

use hal::ieee802154::{self};

use smoltcp::phy::{self, Device, DeviceCapabilities, Medium};
use smoltcp::time::Instant;
use smoltcp::Result;

/// A socket that captures or transmits the complete frame.
//#[derive(Debug)]
pub struct IEEE802154Socket {
    lower: ieee802154::Radio<'static>,
    mtu: usize,
}


impl IEEE802154Socket {
    /// Creates a raw socket, bound to the interface called `name`.
    ///
    /// This requires superuser privileges or a corresponding capability bit
    /// set on the executable.
    pub fn new(radio: ieee802154::Radio<'static>) -> Result<IEEE802154Socket> {
        let mtu = ieee802154::Packet::CAPACITY as usize;
        Ok(IEEE802154Socket {
            lower: radio,
            mtu: mtu,
        })
    }
}

impl<'a> Device<'a> for IEEE802154Socket {
    type RxToken = RxToken;
    type TxToken = TxToken;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit =  self.mtu;
        caps.medium = Medium::Ieee802154;
        caps.max_burst_size = Some(1);
        caps
    }

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let mut lower = &self.lower;
        let mut buffer = vec![0; self.mtu];
        let mut buff = ieee802154::Packet::new();
        match lower.recv(&mut buff) {
            Ok(size) => {
                buffer.resize(size, 0);
                let rx = RxToken { buffer };
                let tx = TxToken {
                    lower: &self.lower,
                };
                Some((rx, tx))
            }
            Err(ref err) if err.kind() == io::ErrorKind::WouldBlock => None,
            Err(err) => panic!("{}", err),
        }
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(TxToken {
            lower: &self.lower,
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
pub struct TxToken {
    lower: &<'a> mut ieee802154::Radio,
}

impl phy::TxToken for TxToken {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        let mut lower = self.lower;
        let mut buffer = vec![0; len];
        let result = f(&mut buffer);
        lower.send(&buffer[..]).unwrap();
        result
    }
}