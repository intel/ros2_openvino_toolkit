   /*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <map>
#include <string>
#include <functional>
#include <memory>
#include "openvino_wrapper_lib/slog.hpp"

template<typename KEY, typename BASE>
class VinoFactory
{
public:
    template<typename T>
	struct TReg
	{
		TReg(KEY key)
		{
			getFactory().getMap().emplace(key, [] { return new T(); });
		}

		template<typename... Args>
		TReg(KEY key, Args&&... args)
		{			
			getFactory().getMap().emplace(key, [&] {return new T(std::forward<Args>(args)...);});
		}
	};

	static BASE* produce(const KEY& key)
	{
        auto item = getFactory().getMap().find(key);

        if(item == getFactory().getMap().end())
        {
			slog::err << "VinoFactory:Invalid inference key: " << key << ". Please check your key name and type!" << slog::endl;
            
			
			slog::info << "-----------Factory Map---------------"  << slog::endl;

			for(auto v : getFactory().getMap())
			{
				slog::info << "  " << v.first << slog::endl;
			}

			slog::info << "-------------------------------------"  << slog::endl;

			return nullptr;
        }
		
        return item->second();
	}

	static std::unique_ptr<BASE> produce_unique(const KEY& key)
	{
		return std::unique_ptr<BASE>(produce(key));
	}

	static std::shared_ptr<BASE> produce_shared(const KEY& key)
	{
		return std::shared_ptr<BASE>(produce(key));
	}

	static BASE* find(const KEY& key)
	{
		auto &instance = getFactory();
		auto item = instance.getMap().find(key);

		if(item == instance.getMap().end())
		{

			slog::err << "VinoFactory: invalid key for find: " << key << ". Please check your key name and type!" << slog::endl;
            
			
			slog::info << "-----------Factory Map---------------"  << slog::endl;

			for(auto v : getFactory().getMap())
			{
				slog::info << "  " << v.first << slog::endl;
			}

			slog::info << "-------------------------------------"  << slog::endl;

			return nullptr;
		}

		return item->second();
	}

private:
	VinoFactory() {};
    virtual ~VinoFactory(){};

    // Disable Left Value Copy
	VinoFactory(const VinoFactory&) = delete;
    VinoFactory &operator=(const VinoFactory&) = delete;

    // Disable Move
	VinoFactory(VinoFactory&&) = delete;

	static VinoFactory<KEY, BASE>& getFactory()
	{
		static VinoFactory<KEY, BASE> instance;
		return instance;
	}

	static std::map<KEY, std::function<BASE*()>>& getMap()
	{
		return getFactory().map_;
	}
	
	std::map<KEY, std::function<BASE*()>> map_;
};