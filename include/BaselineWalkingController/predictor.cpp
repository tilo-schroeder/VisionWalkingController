#pragma once

#include <iostream>
#include <vector>
#include "onnx/include/onnxruntime_cxx_api.h"

class OnnxModel {
public:
    OnnxModel(const std::string& model_path, const int preceding_steps) {
        // Initialize ONNX Runtime environment
        env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "OnnxModel");
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        // Load the model
        session = std::make_unique<Ort::Session>(*env, model_path.c_str(), session_options);

        // Get input dimensions from the model
        Ort::TypeInfo images_type_info = session->GetInputTypeInfo(0);
        auto images_tensor_info = images_type_info.GetTensorTypeAndShapeInfo();
        images_shape = images_tensor_info.GetShape();
        Ort::TypeInfo sides_type_info = session->GetInputTypeInfo(1);
        auto sides_tensor_info = sides_type_info.GetTensorTypeAndShapeInfo();
        sides_shape = sides_tensor_info.GetShape();
        Ort::TypeInfo loopback_type_info = session->GetInputTypeInfo(2);
        auto loopback_tensor_info = loopback_type_info.GetTensorTypeAndShapeInfo();
        loopback_shape = loopback_tensor_info.GetShape();
        Ort::TypeInfo state_type_info = session->GetInputTypeInfo(3);
        auto state_tensor_info = state_type_info.GetTensorTypeAndShapeInfo();
        state_shape = state_tensor_info.GetShape();

        // Initialize prediction buffer
        predictions = std::vector<float>(preceding_steps, 0.0f);
    }

    float process_frame(const std::vector<float>& images) {
        std::vector<std::vector<float>> output = process(images, side, loopback, state);
        loopback = output[1];
        state = output[2];
        return output[0][0];
    }

    void change_side() {
        if (side[0] == 1) {
            side[0] = -1;
        } else {
            side[0] = 1;
        }
    }


private:
    std::unique_ptr<Ort::Env> env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    std::vector<const char*> input_names = {"onnx::Shape_0", "onnx::Gather_1", "onnx::Unsqueeze_2", "onnx::Unsqueeze_3"};
    std::vector<const char*> output_names = {"361", "363", "365"};
    std::vector<int64_t> images_shape;
    std::vector<int64_t> sides_shape;
    std::vector<int64_t> loopback_shape;
    std::vector<int64_t> state_shape;
    std::vector<float> side = {1};
    std::vector<float> loopback = std::vector<float>(32, 0);
    std::vector<float> state = std::vector<float>(32, 0);
    std::vector<float> predictions;

    std::vector<std::vector<float>> process(const std::vector<float> images, const std::vector<float> sides, const std::vector<float> loopback, const std::vector<float> state) {
        
        // debug console output
        //std::cout << "Processing frame with ONNX model..." << std::endl;
        //std::cout << "Input sizes: " << images.size() << " " << sides.size() << " " << loopback.size() << " " << state.size() << std::endl;

        // Create input tensor objects
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value images_tensor = Ort::Value::CreateTensor<float>(memory_info, const_cast<float*>(images.data()), images.size(), images_shape.data(), images_shape.size());
        Ort::Value sides_tensor = Ort::Value::CreateTensor<float>(memory_info, const_cast<float*>(sides.data()), sides.size(), sides_shape.data(), sides_shape.size());
        Ort::Value loopback_tensor = Ort::Value::CreateTensor<float>(memory_info, const_cast<float*>(loopback.data()), loopback.size(), loopback_shape.data(), loopback_shape.size());
        Ort::Value state_tensor = Ort::Value::CreateTensor<float>(memory_info, const_cast<float*>(state.data()), state.size(), state_shape.data(), state_shape.size());
        Ort::Value input_tensors[4] = {std::move(images_tensor), std::move(sides_tensor), std::move(loopback_tensor), std::move(state_tensor)};

        // Run inference
        auto output_tensors = session->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors, 4, output_names.data(), output_names.size());

        // Extract output
        float* outputarr = output_tensors[0].GetTensorMutableData<float>();
        std::vector<float> output(outputarr, outputarr + output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount());
        float* loopbackarr = output_tensors[1].GetTensorMutableData<float>();
        std::vector<float> loopbackOut(loopbackarr, loopbackarr + output_tensors[1].GetTensorTypeAndShapeInfo().GetElementCount());
        float* statearr = output_tensors[2].GetTensorMutableData<float>();
        std::vector<float> stateOut(statearr, statearr + output_tensors[2].GetTensorTypeAndShapeInfo().GetElementCount());

        return {output, loopbackOut, stateOut};
    }
};