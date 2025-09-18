#pragma once

#include <string>

namespace NavigationVI{
    class TextToSpeech{
    public:
        void speak(const std::string& text);
    };
}