#![deny(unused_must_use)]
#![no_main]
#![no_std]

use core::str;
use core::sync::atomic::{self, Ordering};

use cortex_m_rt::entry;
use hal::clocks::{self, Clocks};
use hal::ieee802154::{Channel, TxPower};
use log::{Log};
use smoltcp::iface::{InterfaceBuilder, NeighborCache};
use smoltcp::socket::{SocketSet, UdpPacketMetadata, UdpSocket, UdpSocketBuffer};
use smoltcp::time::{Duration};
use smoltcp::wire::{IpAddress, IpCidr, IpEndpoint};

use core::panic::PanicInfo;
use cortex_m::asm;

use rtt_target::{rprintln, rtt_init_print};

// USB Serial
use hal::usbd::{UsbPeripheral, Usbd};
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use ieee802154socket::IEEE802154Socket;

struct Logger;

impl Log for Logger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= log::STATIC_MAX_LEVEL
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        rprintln!(
            "{}:{} -- {}",
            record.level(),
            record.target(),
            record.args()
        );
    }

    fn flush(&self) {}
}



mod mock {
    use core::cell::Cell;
    use smoltcp::time::{Duration, Instant};

    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Clock(Cell<Instant>);

    impl Clock {
        pub fn new() -> Clock {
            Clock(Cell::new(Instant::from_millis(0)))
        }

        pub fn advance(&self, duration: Duration) {
            self.0.set(self.0.get() + duration)
        }

        pub fn elapsed(&self) -> Instant {
            self.0.get()
        }
    }
}

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
    //rtt_init_print!(BlockIfFull, 16384);
    rtt_init_print!(NoBlockSkip, 16384);
    log::set_logger(&Logger).unwrap();
    log::set_max_level(log::LevelFilter::Trace);

    log::info!("Started…");
    let mock_clock = mock::Clock::new();

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
    let clocks = &*unsafe { CLOCKS.get_or_insert(_clocks) };


    let usb_bus = Usbd::new(UsbPeripheral::new(periph.USBD, &clocks));
    let mut serial = SerialPort::new(&usb_bus);

    let _id = (periph.FICR.deviceid[0].read().bits() as u64) | (periph.FICR.deviceid[1].read().bits() as u64) << 32;
    let id = hexify(_id);
    let short_id: u16 = (_id & 0xffff) as u16;

    // TODO: Hard-coded last two bytes of the hardware ID / IP address of the two ends
    let target_short: u16;
    if short_id == 0xC352 { // DK
        target_short = 0x3805;
    } else {
        target_short = 0xC352;
    }

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

    let device = IEEE802154Socket::new(radio).unwrap();


    let mut neighbor_cache_entries = [None; 8];
    let mut neighbor_cache = NeighborCache::new(&mut neighbor_cache_entries[..]);

    

    let mut rxms = [UdpPacketMetadata::EMPTY; 2];
    let mut rxps: [u8; 256] = [0; 256];
    let udp_rx_buffer = UdpSocketBuffer::new(&mut rxms[..], &mut rxps[..]);

    let mut txms = [UdpPacketMetadata::EMPTY; 2];
    let mut txps: [u8; 256] = [0; 256];
    let udp_tx_buffer = UdpSocketBuffer::new(&mut txms[..], &mut txps[..]);

    let udp_socket = UdpSocket::new(udp_rx_buffer, udp_tx_buffer);

    let ieee802154_addr = smoltcp::wire::Ieee802154Address::Extended(_id.to_be_bytes());

    let mut ip_addrs = [IpCidr::new(
        IpAddress::v6(
            0xfe80, 0, 0, 0, 0x180b, 0x4242, 0x4242, short_id
        ),
        64,
    )];

    /*let target_hwaddr = smoltcp::wire::Ieee802154Address::Extended([
        0x1a, 0x0b, 0x42, 0x42, 0xde, 0xad, 0xbe, 0xef,
    ]);
    */
    let target_ip = IpAddress::v6(
        0xfe80, 0, 0, 0, 0x180b, 0x4242, 0x4242, target_short
    );
    //neighbor_cache.fill( target_ip,smoltcp::wire::HardwareAddress::Ieee802154(target_hwaddr),mock_clock.elapsed());

    let mut ifb = InterfaceBuilder::new(device);

    ifb = ifb.neighbor_cache(neighbor_cache);
    ifb = ifb.hardware_addr(ieee802154_addr.into());
    ifb = ifb.ip_addrs(&mut ip_addrs[..]);
    
    let mut iface = ifb.finalize();

    let mut socket_set_entries: [_; 2] = Default::default();
    let mut sockets = SocketSet::new(&mut socket_set_entries[..]);
    let udp_handle = sockets.add(udp_socket);


/*
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
*/

    let mut x: u16 = 0;
    //let mut receiving = false;
    loop {
        if x == 0 {
            mock_clock.advance(Duration::from_millis(1));
        }
        if x < 1_000 {
            x = x.wrapping_add(1);
        } else {
            x = 0;
        }

        match iface.poll(&mut sockets, mock_clock.elapsed()) {
            Ok(_) => {}
            Err(e) => {
                log::debug!("poll error: {}", e);
            }
        }


        {
            
        let mut socket = sockets.get::<UdpSocket>(udp_handle);
        
        if !socket.is_open() {
            log::debug!("Opening socket…");
            socket.bind(1337).unwrap();
        }

        if socket.can_recv() {
            log::debug!("can_recv");
            socket
                .recv()
                .map(|(data, sender)| {
                    let s = str::from_utf8(data).expect("{{Data not UTF-8}}");
                    log::debug!("Received: {} from {} ({} bytes)", s, sender, data.len());
                    serial.write("Received: ".as_bytes()).ok();
                    serial.write(s.as_bytes()).ok();
                    serial.write(b"\r\n").ok();
                    serial.flush().ok();
                })
                .unwrap_or_else(|e| log::debug!("Recv UDP error: {:?}", e));
        }


        /*
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
        */

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
                                serial.flush().ok();
                            }
                            b'A' => {
                                serial.write(b"\r\nADDR: ").ok();
                                serial.write(&addr).ok();
                                serial.write(b"\r\n").ok();
                                serial.flush().ok();
                            }
                            b'T' => {
                                //if receiving {
                                //    radio.cancel_recv();
                                //    receiving = false;
                                //}
                                //packet.copy_from_slice(b"Hello, World!");
                                //radio.send(&mut packet);
                                //radio.energy_detection_scan(1); // Stop idle TX

                                // udp:6969: respond "hello"
                                let target = IpEndpoint::new(target_ip, 1337);
                                log::info!("{} Attempting to send…", mock_clock.elapsed());
                                match socket.send_slice("Hello, World!".as_bytes(), target) {
                                    Ok(_) => log::info!("OK!"),
                                    Err(err) => {
                                        log::warn!("{}", err);
                                    },
                                }

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

        match iface.poll_delay(&sockets, mock_clock.elapsed()) {
            // Some(Duration { millis: 0 }) => log::debug!("resuming"),
            Some(_delay) => (),/*{
                mock_clock.advance(delay)
            }*/
            None => ()/*mock_clock.advance(Duration::from_millis(1))*/,
        }
    }
}
