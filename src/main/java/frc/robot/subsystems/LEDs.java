// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.util.Color;
// import frc.robot.Constants.LEDConstants;

// public class LEDs {
//     public AddressableLED leds;
//     private final AddressableLEDBuffer buffer;
//     public final int length = LEDConstants.LED_LENGTH;

//     public LEDs() {
//         leds = new AddressableLED(4);
//         buffer = new AddressableLEDBuffer(length);
//         leds.setLength(length);
//         leds.setData(buffer);
//         leds.start();
//     }

//     public void solid(Color color) {
//         if (color != null) {
//             for (int i = 0; i < length; i++) {
//                 buffer.setLED(i, color);
//             }
//         }
//     }

//     public void strobe(Color color, double duration) {
//         boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
//         solid(on ? color : Color.kBlack);
//     }

//     public void laser() {
        
//     }
    

//     public void wave() {}
    
//     public void periodic() {
//         leds.setData(buffer);
//     }
// }
