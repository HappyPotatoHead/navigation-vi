#pragma once

#include <string>

namespace NavigationVI{
    class TextToSpeech{
    public:
        static std::string platformPause(int ms);
        void speak(const std::string& text);
    };
}