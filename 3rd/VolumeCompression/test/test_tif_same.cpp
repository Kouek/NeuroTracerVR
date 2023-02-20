//
// Created by wyz on 2022/3/21.
//
#include <VoxelCompression/utils/TifHelper.hpp>
#include <string>
#include <iostream>
#include <vector>
int main(){
    std::string tif_name1 = "D:/test_1270_merge_max.tif";
    std::string tif_name2 = "D:/volume_tif_to_h264_test/test_01270_lod1.tif";
    TIFInStream tif1;
    TIFInStream tif2;
    tif1.open(tif_name1);
    tif2.open(tif_name2);
    uint32_t w1,h1,bps1,rps1;
    uint32_t w2,h2,bps2,rps2;
    tif1.get(w1,h1,bps1,rps1);
    tif2.get(w2,h2,bps2,rps2);
    std::cerr<<"tif 1: "<<w1<<" "<<h1<<" "<<bps1<<" "<<rps1<<std::endl;
    std::cerr<<"tif 2: "<<w2<<" "<<h2<<" "<<bps2<<" "<<rps2<<std::endl;
    if(w1!=w2 || h1!=h2 || bps1 != bps2){
        exit(-1);
    }
    std::vector<uint8_t> d1(w1);
    std::vector<uint8_t> d2(w1);
    bool same = true;
    for(uint32_t r = 0; r < h1; r++){
        memset(d1.data(),0,w1);
        memset(d2.data(),0,w2);
        tif1.readRow(d1.data(),r,1);
        tif2.readRow(d2.data(),r,1);
        int diff_count = 0;
        for(uint32_t c = 0; c < w1; c++){
            if(d1[c] != d2[c]){
                diff_count++;
            }
        }
        if(diff_count){
            same = false;
            std::cerr<<"row "<<r<<" has "<<diff_count<<" different element"<<std::endl;
        }
    }
    if(same){
        std::cout<<"two tif is the same"<<std::endl;
    }
    return 0;
}