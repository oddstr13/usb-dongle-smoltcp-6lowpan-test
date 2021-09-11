#![deny(unused_must_use)]
#![no_main]
#![no_std]

use core::str;
use core::sync::atomic::{self, Ordering};



use cortex_m_rt::entry;
use hal::clocks::{self, Clocks};
use hal::ieee802154::{self, Channel, Packet, TxPower};

use core::panic::PanicInfo;
use cortex_m::asm;

// USB Serial
use hal::usbd::{UsbPeripheral, Usbd};
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::ieee802154socket;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("{}", info);

    // abort instruction: triggers a HardFault exception which causes probe-run to exit
    asm::udf()
}
static  HEX_CHARS: &[u8; 16] = b"0123456789ABCDEF";
const HEX_SIZE: usize = 24;

fn hexify(mut n: u64) -> [u8; HEX_SIZE] {
    let mut i = 0;
    let mut res: [u8; HEX_SIZE] = [b'\0'; HEX_SIZE];

    while n > 0 {
        if i != 0 {
            res[HEX_SIZE-i-1] = b':';
            i+=1;
        }
        let cur = n & 0xff;

        res[HEX_SIZE-i-1] = HEX_CHARS[(cur&0xf) as usize];
        i += 1;

        res[HEX_SIZE-i-1] = HEX_CHARS[(cur>>4) as usize];
        i += 1;


        n >>= 8;
    }
    //res.reverse();
    res.copy_within(HEX_SIZE-i..HEX_SIZE, 0);
    res[i..HEX_SIZE].fill(b'\0');
    res
}
// D2:03:49:77:39:0A
#[entry]
fn main() -> ! {
    static mut CLOCKS: Option<
        Clocks<clocks::ExternalOscillator, clocks::ExternalOscillator, clocks::LfOscStarted>,
    > = None;
    
    let periph = hal::pac::Peripherals::take().unwrap();
    let clocks = Clocks::new(periph.CLOCK);
    let clocks = clocks.enable_ext_hfosc();
    let clocks = clocks.set_lfclk_src_external(clocks::LfOscConfiguration::NoExternalNoBypass);
    let clocks = clocks.start_lfclk();
    let _clocks = clocks.enable_ext_hfosc();
    //let board = hal::init().unwrap();
    let clocks = unsafe { CLOCKS.get_or_insert(_clocks) };


    let usb_bus = Usbd::new(UsbPeripheral::new(periph.USBD, &clocks));
    let mut serial = SerialPort::new(&usb_bus);

    let _id = (periph.FICR.deviceid[0].read().bits() as u64) | (periph.FICR.deviceid[1].read().bits() as u64) << 32;
    let id = hexify(_id);

    let _addr = (periph.FICR.deviceaddr[0].read().bits() as u64) | ((periph.FICR.deviceaddr[1].read().bits() & 0xffff) as u64) << 32;
    let addr = hexify(_addr);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("OpenShell.no")
        .product("usb-dongle-smoltcp-6lowpan-test")
        .serial_number(str::from_utf8(&addr).unwrap_or(""))
        .device_class(USB_CLASS_CDC)
        .device_release(0x0001)
        .max_packet_size_0(64) // (makes control transfers 8x faster)
        .build();

    let mut radio = hal::ieee802154::Radio::init(periph.RADIO, &clocks);

    // these are the default settings of the DK's radio
    // NOTE if you ran `change-channel` then you may need to update the channel here
    radio.set_channel(Channel::_20); // <- must match the Dongle's listening channel
    radio.set_txpower(TxPower::Pos8dBm);

    let mut packet = Packet::new();

    // these three are equivalent
    // let msg: &[u8; 5] = &[72, 101, 108, 108, 111];
    // let msg: &[u8; 5] = &[b'H', b'e', b'l', b'l', b'o'];
    let mut _msg = [b'H', b'e', b'l', b'l', b'o', b' ', b' '];
    let msg: &mut[u8] = &mut _msg[..];

    log::info!(
        "sending: {}",
        str::from_utf8(msg).expect("msg is not valid UTF-8 data")
    );

    for n in 0..=9 {

        msg[6] = n + 48;

        packet.copy_from_slice(msg);
        radio.send(&mut packet);
    }

    // Turn off TX for the love of the spectrum!
    //radio.energy_detection_scan(1);


    let mut receiving = false;

    loop {
        if !receiving {
            radio.recv_async_start(&mut packet);
            receiving = true;
        } else if radio.recv_async_poll() {
            let res = radio.recv_async_sync();
            receiving = false;
            match res {
                Ok(_crc) => {
                    serial.write(b"Received: ").ok();
                    serial.write(str::from_utf8(&*packet).expect("Data not UTF-8").as_bytes()).ok();
                    serial.write(b"\r\n").ok();
                },
                Err(_crc) => {
                    serial.write(b"RX failed\r\n").ok();
                },
            }
        }

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if 0x61 <= *c && *c <= 0x7a {
                            *c &= !0x20;
                        }
                        // Stop on receiving Q
                        match *c {
                            b'Q' => {
                                serial.write(b"\r\nEXITING\r\n").ok();
                                serial.flush().ok();
                                // force any pending memory operation to complete before the BKPT instruction that follows
                                atomic::compiler_fence(Ordering::SeqCst);
                                asm::bkpt()
                            }
                            b'I' => {
                                serial.write(b"\r\nID: ").ok();
                                serial.write(&id).ok();
                                serial.write(b"\r\n").ok();
                            }
                            b'A' => {
                                serial.write(b"\r\nADDR: ").ok();
                                serial.write(&addr).ok();
                                serial.write(b"\r\n").ok();
                            }
                            b'T' => {
                                if receiving {
                                    radio.cancel_recv();
                                    receiving = false;
                                }
                                packet.copy_from_slice(b"Hello, World!");
                                radio.send(&mut packet);
                                //radio.energy_detection_scan(1); // Stop idle TX
                            }
                            _ => ()
                        }
                    }

                    let mut write_offset = 0;
                    while write_offset < count {
                        match serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }

    }
}
