// This code was written by me, zenith-parsec, and ChatGPT using the GPT-4o model.
// Not all code was written by ChatGPT, and I made lots of changes myself to
// the code to get it to work the way I wanted.
//
// For a transcript of the chat that resulted in most of the early code (first commit)
// look at:
//
//     https://chatgpt.com/share/67132632-9b8c-8000-8483-046ebea0f9ee
//
// Testing was performed using "the arena floor", a 3lb class combat robot. (It has lost two fights, won zero. Do not judge based on numbers. Judge based on me saying it's a terrible robot.)
//
// - FPVDrone Fli14+ 14CH Mini Receiver 2A
// https://www.amazon.com/dp/B086W6LG8Z
// - Adafruit H3LIS331 accelerometer
// https://www.adafruit.com/product/4627
// - 2 x Repeat AM32 Drive ESC
// https://repeat-robotics.com/buy/am32/
// - Teensy 4.0 (which runs this code)
// https://www.pjrc.com/store/teensy40.html
// - FlySky FS-i6 transmitter
// available everywhere. I got mine on Amazon, but the listing I used was only reasonably priced because it was on sale. not linking it.
// = red, green, and blue indicator LEDs. one each of for top (and bottom if you are invertible)
// - 2 x brushless motors (I'm embarrassed to tell you how big mine are. Or where I got them. You don't need to be like me.)
// - 2 x wheels and mounting hardware
// [what I used] https://itgresa.com/product/banebots-t81-wheels/ and https://itgresa.com/product/banebots-t81-hubs/
//   - these are quite heavy wheels, but they get good grip. I got 50A for my bot. They work, but there are probably lighter wheels that perform better.
// - an approptiate battery
//   - 3S 750mAh LiPo seems ok so far?
//   - https://www.amazon.com/dp/B097BWZTVW hasn't caught fire yet.
// - a body to mount these things on.
//   - CAD done by me in the US. Milling done by JLCPCB in China.
//     - CAD was cheap
//     - getting parts machined when you have zero machining knowledge is not cheap.
//       - definitely watch some YouTube videos on it. Or don't do it, and do something in TPU. I don't care.
// - various connectors and tools
//   - soldering iron and consumables.
//   - wires of various thickness, along with ways to trim and strip them.
//   - appropriate connectors for ESC to motors.
//   - screws/bolts/etc and tools
//   - a USB mini cable for the Teensy.
//   - a pair of wheel clip pliers.
//   - a multimeter/oscilliscope/other way of detecting voltage
//   - a computer with the Internet and a USB port. (not always essential, but may help if you think you can fix something in software)
// - patience and humility.
//   - this will not work first try.
//     - this may or may not be your fault.
//   - violence is not the answer
//     - while your robot might be designed to be tough and resilient, it's best not to see how it deals with being thrown across the room.
//       - you may be wrong about how strong it is.
//       - it may land on a weak point.
//       - your room might not be as strong as your robot.

// Design and use tips

// The receiver should connect to RX2 (pin 7) of the Teensy and a common ground pin.
// The signal pins from the ESCs should be connected to pins 2 and 3, and the ground
// pins to ground.

// I use a custom circuit board to hold things together, but it's still evolving
// so I won't share it yet.

// The robot should have red, green, and blue LEDs (all with resistors) on -ve side
// of the appropriately named ports. The other side should be connected to 5v.
// The logic driving the LEDs in updateLEDs is inverted, with literal 1 turning the
// LED off, and a 0 turning it on. Red LEDs have fwd voltage of about 1.6-2.2v, but
// we are connecting the negative lead to 3.3v, and the positive lead to 5v. With
// the negative lead supplying 3.3v, the potential difference over the LED is only
// 1.7v, which isn't enough to brightly light up a red LED, and nowhere near enough
// to light up a green or blue one. When the LED pin goes to 0v, the difference
// becomes 5v, and all the LEDs are brightly illuminated. This is probably bad
// for the Teensy.

// the IBus connection needs a hardware serial port. The IBusBM library doesn't
// support Teensy by default: it does compile, but it sends a warning message.
// I couldn't easily add perfect support, but I removed the warning part in the
// begin() method about being an unsupported board so it compiles cleanly, and
// I call the ibus.loop() method manually. (it might already be called on the timer
// I added as well, but I have no idea how to tell at the moment.) It's not like
// this makes the code any worse.
