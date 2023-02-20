//
// Created by wyz on 2022/3/18.
//
#include <tiffio.h>
#include <string>
int main(){
    std::string path = "F:/202790_06461_CH1.tif";
    TIFF* tif = TIFFOpen(path.c_str(), "r");
    if (tif) {
        uint32 w, h;
        size_t npixels;
        uint32* raster;

        TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
        TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);
        npixels = w * h;
        raster = (uint32*) _TIFFmalloc(npixels * sizeof (uint32));
        if (raster != NULL) {
            if (TIFFReadRGBAImage(tif, w, h, raster, 0)) {

            }
            _TIFFfree(raster);
        }
        TIFFClose(tif);
    }
}