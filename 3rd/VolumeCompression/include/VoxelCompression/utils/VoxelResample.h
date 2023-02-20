//
// Created by wyz on 2021/4/8.
//
//
//
/**
 * @file using this should link library tinyTIFF and add "#define VoxelResampleIMPL" before include this file
 */

#pragma once

#ifdef VoxelResampleIMPL
#include<fstream>
#include<thread>
#include<atomic>
#include<condition_variable>
#include<functional>
#include<cassert>
#include<iostream>
#include <utility>
#include<vector>
#include <iomanip>
#include <sstream>
#include<VoxelCompression/utils/util.h>

class Worker {
public:
    Worker() : busy(false) {}

    Worker(const Worker &worker) : busy(worker.busy.load()) {}

    bool is_busy() const {
        return busy;
    }

    template<class T>
    void setup_task(size_t x, size_t y, std::vector<T> &data, std::function<T(T, T)> method) {
        assert(busy);
        assert(x * y * 2 == data.size());
        size_t slice_size = x * y;

        //z-direction
        for (size_t i = 0; i < slice_size; i++) {
            data[i] = method(data[i], data[i + slice_size]);
        }
        //y-direction
        data.resize(slice_size);

        size_t modify_y = (y + 1) / 2;
        size_t modify_slice_y = modify_y * x;
        data.resize(modify_slice_y * 2, 0);

        for (size_t i = 0; i < modify_y; i++) {
            for (size_t j = 0; j < x; j++) {
                data[j + i * x] = method(data[j + i * x * 2], data[j + x + i * x * 2]);
            }
        }
        //x-direction
        data.resize(modify_slice_y);

        size_t modify_x = (x + 1) / 2;
        size_t modify_slice_yx = modify_x * modify_y;
//        data.resize(modify_slice_yx * 2, 0);

        for (size_t i = 0; i < modify_y; i++) {
            for (size_t j = 0; j < modify_x; j++)
                data[j + i * modify_x] = method(data[i * x + j * 2], j * 2 + 1 >= x ? 0 : data[i * x + j * 2 + 1]);
        }
        data.resize(modify_slice_yx);

    }

    void set_busy(bool status) { busy = status; }

private:
    std::atomic<bool> busy;

};

class VolumeResampler {
public:
    enum class ResampleMethod {
        AVG, MAX
    };
    enum class InputFormat {
        RAW,
        TIF
    };

    VolumeResampler(const std::string &input, InputFormat input_format, std::string prefix, const std::string &output,
                    int raw_x, int raw_y, int raw_z,
                    int memory_limit, ResampleMethod method = ResampleMethod::MAX) noexcept
            : input_path(input), input_format(input_format), prefix(std::move(prefix)), raw_x(raw_x), raw_y(raw_y),
              raw_z(raw_z), memory_limit(memory_limit), resample_times(2),
              method(method) {
        if (input_format == InputFormat::RAW) {
            in = std::ifstream(input.c_str(), std::ios::binary);
            if (!in.is_open()) {
                throw std::runtime_error("input file open failed");
            }
        }
        //write to single file
        out = std::ofstream(output.c_str(), std::ios::binary);
        if (!out.is_open()) {
            throw std::runtime_error("output file open failed");
        }


        int payload = (int64_t) raw_x * raw_y * resample_times / 1024/*KB*// 1024/*MB*// 1024/*GB*/;
        payload = payload > 0 ? payload : 1;
        int num = memory_limit / payload;
        SYSTEM_INFO sys_info;
        GetSystemInfo(&sys_info);
        int cpu_core_num = sys_info.dwNumberOfProcessors / 2;
        std::cout << "cpu core num is: " << cpu_core_num << std::endl;
        workers.resize(min(num, cpu_core_num * 3 / 4));
        tasks.resize(workers.size());
    }

    void print_args() const {
        std::cout << "[VolumeResampler args]:"
                  << "\n\traw_xyz: " << raw_x << " " << raw_y << " " << raw_z
                  << "\n\tmemory_limit: " << memory_limit
                  << "\n\tresample_times: " << resample_times
                  << "\n\tmethod: " << (method == ResampleMethod::MAX ? "max" : "average")
                  << "\n\tworkers num: " << workers.size() << std::endl;
    }

    void start_task() {
        int total_task_num = (raw_z + resample_times - 1) / resample_times;
        std::atomic<int> current_task_idx(0);
        std::mutex mtx;
        std::condition_variable worker_cv;
        std::mutex io_mtx;
        std::condition_variable io_cv;
        while (current_task_idx < total_task_num) {
            //first wait for available worker
            {
                std::unique_lock<std::mutex> lk(mtx);
                worker_cv.wait(lk, [&]() {
                    for (auto &worker:workers) {//std::any_of ???
                        if (!worker.is_busy()) {
                            return true;
                        }
                    }
                    return false;
                });
            }
            //second assign task to available worker
            for (int i = 0; i < workers.size(); i++) {
                if (!workers[i].is_busy()) {
                    int work_idx = current_task_idx++;
                    if (work_idx >= total_task_num) {
                        break;
                    }
                    printf("start worker: %d, about finish %.2f%%\n", work_idx, work_idx * 1.f / total_task_num * 100);
                    workers[i].set_busy(true);
                    if (tasks[i].joinable())
                        tasks[i].join();
                    tasks[i] = std::thread([&](Worker &worker_, int id) {
                        std::vector<uint8_t> payload;
                        size_t payload_size = (size_t) raw_x * raw_y * resample_times;
                        payload.resize(payload_size, 0);
                        //read
                        {
                            std::unique_lock<std::mutex> read_lk(io_mtx);
                            io_cv.wait(read_lk, []() { return true; });

                            if(input_format==InputFormat::RAW){

                                in.seekg(std::ios::beg + payload_size * id);

                                in.read(reinterpret_cast<char *>(payload.data()), payload_size);
                            } else if (input_format == InputFormat::TIF) {
                                for (int j = 0; j < resample_times; j++) {
                                    std::stringstream ss;
                                    ss<<input_path<<"/"<<prefix<<std::setw(std::ceil(log10(raw_z)))<<std::setfill('0')<<std::to_string(id*resample_times+j+1)<<".tif";
//                                    ss<<input_path<<"/"<<prefix<<std::setw(5)<<std::setfill('0')<<std::to_string(id*resample_times+j+1)<<".tif";
                                    std::vector<uint8_t> slice;
                                    load_volume_tif(slice, ss.str());
                                    if (slice.size() == (size_t) raw_x * raw_y) {
                                        memcpy(payload.data() + j * slice.size(), slice.data(), slice.size());
                                    } else {
                                        throw std::runtime_error("load tif slice size not equal to raw_x*raw_y");
                                    }
                                }
                            }

                            io_cv.notify_one();
                        }

                        //resample
                        if(method==ResampleMethod::MAX){
                          worker_.setup_task<uint8_t>(raw_x, raw_y, payload, [](uint8_t a, uint8_t b) -> uint8_t {
                            return a > b ? a : b;
                          });
                        }
                        else if(method==ResampleMethod::AVG){
                          worker_.setup_task<uint8_t>(raw_x, raw_y, payload, [](uint8_t a, uint8_t b) -> uint8_t {
                            return (a + b) / 2;
                          });
                        }

                        //write
                        {
                            std::unique_lock<std::mutex> write_lk(io_mtx);
                            io_cv.wait(write_lk, []() { return true; });
                            out.seekp(std::ios::beg + payload.size() * id);

                            out.write(reinterpret_cast<char *>(payload.data()), payload.size());
                            io_cv.notify_one();
                        }
                        worker_.set_busy(false);
                        worker_cv.notify_one();

                    }, std::ref(workers[i]), work_idx);


                }
            }

        }
        for (int i = 0; i < tasks.size(); i++) {
            if (tasks[i].joinable())
                tasks[i].join();
        }
        in.close();
        out.close();
        std::cout << "successfully finish task!" << std::endl;
    }

private:
    std::string input_path;
    InputFormat input_format;
    std::string prefix;
    int raw_x, raw_y, raw_z;
    int memory_limit;
    int resample_times;
    ResampleMethod method;
    std::ifstream in;
    std::ofstream out;
    std::vector<Worker> workers;
    std::vector<std::thread> tasks;
};
#endif

