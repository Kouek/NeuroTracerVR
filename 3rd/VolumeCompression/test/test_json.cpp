//
// Created by wyz on 2021/4/7.
//
#include<VoxelCompression/voxel_compress/VoxelCmpDS.h>
#include<tinytiffreader.h>
void test_write(){
    sv::LodFile lod_file;

    lod_file.add_lod_file(0,"lod0.h264");
    lod_file.add_lod_file(1,"lod1.h264");
    lod_file.add_lod_file(2,"lod2.h264");
    lod_file.add_lod_file(3,"lod3.h264");
    lod_file.add_lod_file(4,"lod4.h264");
    lod_file.save_lod_files("test_json","json");

}
void test_read(){
    sv::LodFile lod_file;
    lod_file.open_lod_file("test_json.json");
    std::cout<<"min_lod: "<<lod_file.get_min_lod()<<" max_lod: "<<lod_file.get_max_lod()<<std::endl;
    for(int i=lod_file.get_min_lod();i<=lod_file.get_max_lod();i++){
        std::cout<<std::to_string(i)<<": "<<lod_file.get_lod_file_path(i)<<std::endl;
    }
}
void test_tif(){
    TinyTIFFReaderFile* tiffr= nullptr;
    tiffr=TinyTIFFReader_open("test_00001.tif");
    if (!tiffr) {
        std::cout<<"    ERROR reading (not existent, not accessible or no TIFF file)\n";
    } else {
        if (TinyTIFFReader_wasError(tiffr)) {
            std::cout<<"   ERROR:"<<TinyTIFFReader_getLastError(tiffr)<<"\n";
        } else {
            std::cout<<"    ImageDescription:\n"<< TinyTIFFReader_getImageDescription(tiffr) <<"\n";
            uint32_t frames=TinyTIFFReader_countFrames(tiffr);
            std::cout<<"    frames: "<<frames<<"\n";
            uint32_t frame=0;
            if (TinyTIFFReader_wasError(tiffr)) {
                std::cout<<"   ERROR:"<<TinyTIFFReader_getLastError(tiffr)<<"\n";
            } else {
                do {
                    const uint32_t width=TinyTIFFReader_getWidth(tiffr);
                    const uint32_t height=TinyTIFFReader_getHeight(tiffr);
                    const uint16_t samples=TinyTIFFReader_getSamplesPerPixel(tiffr);
                    const uint16_t bitspersample=TinyTIFFReader_getBitsPerSample(tiffr, 0);
                    bool ok=true;
                    std::cout<<"    size of frame "<<frame<<": "<<width<<"x"<<height<<"\n";
                    std::cout<<"    each pixel has "<<samples<<" samples with "<<bitspersample<<" bits each\n";
                    if (ok) {
                        frame++;
                        // allocate memory for 1 sample from the image
                        uint8_t* image=(uint8_t*)calloc(width*height, bitspersample/8);

                        for (uint16_t sample=0; sample<samples; sample++) {
                            // read the sample
                            TinyTIFFReader_getSampleData(tiffr, image, sample);
                            if (TinyTIFFReader_wasError(tiffr)) { ok=false; std::cout<<"   ERROR:"<<TinyTIFFReader_getLastError(tiffr)<<"\n"; break; }

                            ///////////////////////////////////////////////////////////////////
                            // HERE WE CAN DO SOMETHING WITH THE SAMPLE FROM THE IMAGE
                            // IN image (ROW-MAJOR!)
                            // Note: That you may have to typecast the array image to the
                            // datatype used in the TIFF-file. You can get the size of each
                            // sample in bits by calling TinyTIFFReader_getBitsPerSample() and
                            // the datatype by calling TinyTIFFReader_getSampleFormat().
                            ///////////////////////////////////////////////////////////////////

                        }

                        free(image);
                    }
                } while (TinyTIFFReader_readNext(tiffr)); // iterate over all frames
                std::cout<<"    read "<<frame<<" frames\n";
            }
        }
    }
    TinyTIFFReader_close(tiffr);
}
void test_tf_read(){
    json j;
    std::ifstream in("tf.json");
    in>>j;
    int point_num=j["point_num"];
    auto points=j["points"];
    auto p=points["0"];
    double r=p[0];
    double g=p[1];
    double b=p[2];
    double a=p[3];
    std::cout<<r<<" "<<g<<" "<<b<<" "<<a<<std::endl;
}
int main(int argc,char** argv)
{

    return 0;
}
