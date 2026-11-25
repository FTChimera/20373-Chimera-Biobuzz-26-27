package org.firstinspires.ftc.teamcode.fakeHw;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Generic RGBA + distance stimulus, not a concrete REV/I2C vendor sensor. */
public final class FakeColorSensor extends FakeDistanceSensor implements ColorSensor, NormalizedColorSensor {
    private static final class ColorReading extends NormalizedRGBA {
        @Override public int toColor() {
            return (packed(alpha) << 24) | (packed(red) << 16) | (packed(green) << 8) | packed(blue);
        }
        private static int packed(float value) {
            return Math.min(255, (int) (clip(value, 0, 1) * 256));
        }
    }
    private int red, green, blue, alpha;
    private float normalizedRed, normalizedGreen, normalizedBlue, normalizedAlpha;
    private float gain = 1;
    private boolean led;
    private I2cAddr address;
    private final List<Boolean> ledWrites = new ArrayList<>();
    private final List<Float> gainWrites = new ArrayList<>();
    private final List<I2cAddr> addressWrites = new ArrayList<>();

    public void setColor(int red, int green, int blue, int alpha) {
        requireOpen();
        channel(red); channel(green); channel(blue); channel(alpha);
        this.red = red; this.green = green; this.blue = blue; this.alpha = alpha;
        normalizedRed = red / 255f; normalizedGreen = green / 255f;
        normalizedBlue = blue / 255f; normalizedAlpha = alpha / 255f;
    }
    private static void channel(int value) {
        if (value < 0 || value > 255) throw new IllegalArgumentException("raw channels must be in [0,255]");
    }
    public void setNormalizedColors(float red, float green, float blue, float alpha) {
        requireOpen();
        normalized(red); normalized(green); normalized(blue); normalized(alpha);
        normalizedRed = red; normalizedGreen = green; normalizedBlue = blue; normalizedAlpha = alpha;
        this.red = Math.round(red * 255); this.green = Math.round(green * 255);
        this.blue = Math.round(blue * 255); this.alpha = Math.round(alpha * 255);
    }
    private static void normalized(float value) {
        finite(value, "normalized channel");
        if (value < 0 || value > 1) throw new IllegalArgumentException("normalized channels must be in [0,1]");
    }
    @Override public int red() { requireOpen(); return red; }
    @Override public int green() { requireOpen(); return green; }
    @Override public int blue() { requireOpen(); return blue; }
    @Override public int alpha() { requireOpen(); return alpha; }
    @Override public int argb() { requireOpen(); return (alpha << 24) | (red << 16) | (green << 8) | blue; }
    @Override public NormalizedRGBA getNormalizedColors() {
        requireOpen();
        NormalizedRGBA value = new ColorReading();
        value.red = (float) clip((double) normalizedRed * gain, 0, 1);
        value.green = (float) clip((double) normalizedGreen * gain, 0, 1);
        value.blue = (float) clip((double) normalizedBlue * gain, 0, 1);
        value.alpha = (float) clip((double) normalizedAlpha * gain, 0, 1);
        return value;
    }
    @Override public float getGain() { requireOpen(); return gain; }
    @Override public void setGain(float gain) {
        requireOpen();
        finite(gain, "gain");
        if (gain < 0) throw new IllegalArgumentException("gain must be nonnegative");
        this.gain = gain;
        gainWrites.add(gain);
    }
    @Override public void enableLed(boolean enable) { requireOpen(); led = enable; ledWrites.add(enable); }
    public boolean isLedEnabled() { requireOpen(); return led; }
    @Override public void setI2cAddress(I2cAddr address) {
        requireOpen();
        Objects.requireNonNull(address, "address");
        if (address.get8Bit() < 0 || address.get8Bit() > 255) {
            throw new IllegalArgumentException("I2C address must fit in eight bits");
        }
        this.address = address;
        addressWrites.add(address);
    }
    @Override public I2cAddr getI2cAddress() {
        requireOpen();
        if (address == null) throw new IllegalStateException("No I2C address configured");
        return address;
    }
    public List<Boolean> getLedHistory() { return snapshot(ledWrites); }
    public List<Float> getGainHistory() { return snapshot(gainWrites); }
    public List<I2cAddr> getAddressHistory() { return snapshot(addressWrites); }
    public int getLedWriteCount() { return ledWrites.size(); }
    public int getGainWriteCount() { return gainWrites.size(); }
    public int getAddressWriteCount() { return addressWrites.size(); }
    @Override public void resetDeviceConfigurationForOpMode() {
        super.resetDeviceConfigurationForOpMode();
        gain = 1;
        led = false;
    }
}
