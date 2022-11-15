package frc.libs.sdsLib.ctre;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.libs.sdsLib.AbsoluteEncoder;
import frc.libs.sdsLib.AbsoluteEncoderFactory;

public class MagEncoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;

    public MagEncoderFactoryBuilder withReadingUpdatePeriod() {
        return this;
    }

    public AbsoluteEncoderFactory<MagEncoderAbsoluteConfiguration> build() {
        return configuration -> {
            OffsetMagEncoder encoder = new OffsetMagEncoder(new DutyCycle(new DigitalInput(configuration.getId())), configuration.getOffset());
            encoder.setDistancePerRotation(360 * (direction == Direction.COUNTER_CLOCKWISE ? 1 : -1)); // TODO check direction
            // i have also seen values here as (1.0/4098.0, 4096.0/4098.0) as that might edge it touch towards the bounds
            encoder.setDutyCycleRange(1.0/4096.0, 4095.0/4096.0); 
            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final OffsetMagEncoder encoder;

        private EncoderImplementation(OffsetMagEncoder encoder) {
            this.encoder = encoder;
        }

        /**
         * Get Absolute angle of sensor in Radians, overriden to provide radians 0 to 2pi
         *
         */
        @Override
        public double getAbsoluteAngle() {
           // double angle = Math.toRadians(encoder.getDistance()); wrong method for absolute position of SRX when wired for DIO/might be correct if used in quadrature mode.
            double angle = Math.toRadians(encoder.getAbsolutePosition()); // getAbsolutePosition is correct for SRX in DIO absolute mode
            angle %= 2.0 * Math.PI; // overiding so we can return radians I see.  if this returns max 39.4 instead of max 6.28 then getAbsolutePosition was overridden to return radians already.
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
