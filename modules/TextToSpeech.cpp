#include "TextToSpeech.h"

#include <cstdlib>

namespace NavigationVI{
    void TextToSpeech::speak(const std::string& text){
        #ifdef _WIN32
            std::string command = "PowerShell -Command \"Add-Type -AssemblyName System.Speeh; " 
                                 "(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('" + text "');\"";
        #elif __APPLE__
            std::string command = "say \"" + text + "\"";
        #else  
            std::string command = "espeak \"" + text + "\"";
        #endif
            std::system(command.c_str());
    }
}