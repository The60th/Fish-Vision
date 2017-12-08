package org.firstinspires.ftc.teamcode.cv_libs;

/**
 * Created by FTC 8397 on 11/6/2016.
 *
 * <p>
 * Provides static image processing methods.
 */

public class ImgProc {

    /*
    getReducedRangeRGB565
    This method combines the functionality of getReducedRGB565 and getRangeRGB565

    RGB565 source image is passed in as src, with its width and height passed in as srcWidth, srcHeight.

    A subrange of the source image is extracted, with upper left corner at x0,y0 of source image, and
    subrange width and height of rangeWidth, rangeHeight. If the sampleRatio argument is 1, then
    this subrange is used without further reduction.

    The resolution of the subrange may be further reduced by passing in sampleRatio > 1. Then, within
    the specified subrange, only every "sampleRatio'th" row and column will be sampled. It is required
    that sampleRatio be a common divisor of rangeWidth and rangeHeight.

    The dimensions of the final width of the destination image will be rangeWidth/sampleRatio, and
    its height will be rangeHeight/sampleRatio.

     */

    public static void getReducedRangeRGB565(byte[] src, int srcWidth, int srcHeight, int x0, int y0,
                                             int rangeWidth, int rangeHeight, byte[] dst, int sampleRatio)
    {
        int widthRatio = sampleRatio;
        int heightRatio = sampleRatio;
        int dstWidth = rangeWidth / sampleRatio;
        int dstHeight = rangeHeight / sampleRatio;

        for (int y = 0; y < dstHeight; y++)
        {
            int srcRowIndex = 2 * srcWidth * (y0 + heightRatio * y);
            for (int x = 0; x < dstWidth; x++)
            {
                int srcIndex = srcRowIndex + 2 * (x0 + widthRatio * x);
                int dstIndex = 2 * (y * dstWidth + x);
                dst[dstIndex] = src[srcIndex];
                dst[dstIndex + 1] = src[srcIndex + 1];
            }
        }
    }


    /*
    getBinaryImage
    Compares each pixel of an RGB565 image with the accepted range of hue, saturation, and value,
    and creates a new "Binary Image"--an array of integers with values of 0 (no match) or 1 (match).

    Source image is passed in as src.

    Note: for target colors near red, which has hue of 0, the values used for minHue and maxHue may not be intuitive.
    For example, to bracket a hue of 0 +/- 30, you would use minHue of 330 and maxHue of 30.

     */
    public static void getBinaryImage(byte[] src, float minHue, float maxHue, float minSat,
                                      float maxSat, float minVal, float maxVal, int[] dst) {
        int red, green, blue;
        int max, min, c; //These will ultimately hold max(r,g,b), min(r,g,b) and chroma, which is max-min
        int numPixels = src.length / 2; //RGB565 stores each pixel as two bytes
        int i1, i2;
        float hPrime;
        //Use min and max hue in the 0-6 range to avoid repeated multiplication by 60f
        float minHue06 = minHue / 60.0f;
        float maxHue06 = maxHue / 60.0f;
        //Use integer min and max value in the 0-255 range to avoid repeated division by 255f
        int minVal255 = Math.round(minVal * 255.0f);
        int maxVal255 = Math.round(maxVal * 255.0f);
        float sat;

        for (int i = 0; i < numPixels; i++) {
            dst[i] = 0; //Set the binary destination pixel to 0; we'll change it to 1 if it proves a match
            i1 = 2 * i;
            i2 = 2 * i + 1;

            //Extract the red, green, and blue values from the RGB565 bytes for this pixel
            blue = (src[i1] & 0x1F) << 3;
            red = (src[i2] & 0xF8);
            green = ((src[i1] & 0xE0) >> 3) + ((src[i2] & 0x7) << 5);

            //Do in-line conversion from RGB565 to HSV to avoid repeated function calls
            //Note: Rather than computing hue in the 0-360 range, will use hPrime in the 0 to 6 range
            //Actual hue would be obtained by multiplying by 60, but that would be wasteful

            if (red > green) {
                if (red > blue) {
                    //Red is Maximum, either Green or Blue may be Minimum
                    max = red;
                    min = Math.min(green, blue);
                    c = max - min;
                    hPrime = c == 0 ? 0 : (float) (green - blue) / (float) c + 6.0f;
                    hPrime = hPrime >= 6.0f ? hPrime - 6.0f : hPrime;
                } else {
                    //Blue is Maximum, Green is Minimum
                    max = blue;
                    c = max - green;
                    hPrime = c == 0? 0 : (float) (red - green) / (float) c + 4.0f;
                }
            } else {
                if (green > blue) {
                    //Green is Maximum, either Red or Blue may be Minimum
                    max = green;
                    min = Math.min(red, blue);
                    c = max - min;
                    hPrime = c == 0? 0 : (float) (blue - red) / (float) c + 2.0f;
                } else {
                    //Blue is Maximum, Red is Minimum
                    max = blue;
                    c = max - red;
                    hPrime = c == 0? 0 : (float) (red - green) / (float) c + 4.0f;
                }
            }

            //Check if hue is a match. If not, move on to the next pixel
            if ( (maxHue >= minHue && (hPrime < minHue06 || hPrime > maxHue06))
                    || (maxHue < minHue && hPrime > maxHue06 && hPrime < minHue06) ) continue;

            //Saturation is just the chroma divided by the max value among r,g,b (unless the max value is 0)
            sat = max > 0 ? (float) c / (float) max : 0;

            //Check if saturation and value are a match. If not, move on to the next pixel
            if (sat < minSat || sat > maxSat || max < minVal255 || max > maxVal255) continue;

            //To reach this point, hue, saturation, and value must all match. Set binary pixel to 1.
            dst[i] = 1;
        }
    }

    public static void getBinaryImage(byte[] src, HSV_Range hsvRange, int[] dst){
        getBinaryImage(src, hsvRange.getMinHue(), hsvRange.getMaxHue(), hsvRange.getMinSat(), hsvRange.getMaxSat(),
                hsvRange.getMinVal(), hsvRange.getMaxVal(), dst);
    }

}
