/*
// Copyright (c) 2017-2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#pragma once

#include <ie_iextension.h>

#include <string>
#include <vector>

namespace InferenceEngine {
namespace Extensions {
namespace Cpu {

class ExtLayerBase: public ILayerExecImpl {
public:
    explicit ExtLayerBase(const CNNLayer *layer): cnnLayer(*layer) {}

    StatusCode getSupportedConfigurations(std::vector<LayerConfig>& conf, ResponseDesc *resp) noexcept override;
    StatusCode init(LayerConfig& config, ResponseDesc *resp) noexcept override;

protected:
    enum class ConfLayout { ANY, PLN, BLK8, BLK16 };

    class DataConfigurator {
    public:
        explicit DataConfigurator(ConfLayout l):
            layout(l) {}

        DataConfigurator(ConfLayout l, bool constant, int inplace = -1):
                layout(l), constant(constant), inplace(inplace) {}

        ConfLayout layout;
        bool constant = false;
        int inplace = -1;
    };

    void addConfig(std::vector<DataConfigurator> in_l, std::vector<DataConfigurator> out_l, bool dynBatchSupport = false);
    std::string errorMsg;
    CNNLayer cnnLayer;
    std::vector<LayerConfig> confs;
};

template <class IMPL>
class ImplFactory : public ILayerImplFactory {
public:
    explicit ImplFactory(const CNNLayer *layer): cnnLayer(*layer) {}

    StatusCode getShapes(const std::vector<TensorDesc>& inShapes, std::vector<TensorDesc>& outShapes,
            ResponseDesc *resp) noexcept override {
        return NOT_IMPLEMENTED;
    }

    // First implementation has more priority than next
    StatusCode getImplementations(std::vector<ILayerImpl::Ptr>& impls, ResponseDesc *resp) noexcept override {
        impls.push_back(ILayerImpl::Ptr(new IMPL(&cnnLayer)));
        return OK;
    }

protected:
    CNNLayer cnnLayer;
};

}  // namespace Cpu
}  // namespace Extensions
}  // namespace InferenceEngine
