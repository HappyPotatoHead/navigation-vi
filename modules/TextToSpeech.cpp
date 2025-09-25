#include "TextToSpeech.h"

#include <cstdlib>

namespace NavigationVI{
    std::string TextToSpeech::platformPause(int ms) {
    #ifdef _WIN32
        // SAPI supports SSML <break>
        return "<break time='" + std::to_string(ms) + "ms'/>";
    #elif __APPLE__
        // macOS 'say' doesn't support SSML or slnc — just return empty
        return "";
    #else
        // eSpeak pause syntax
        return "[[slnc " + std::to_string(ms) + "]]";
    #endif
    }
    void TextToSpeech::speak(const std::string& text){
        #ifdef _WIN32
            std::string command = "PowerShell -Command \"Add-Type -AssemblyName System.Speech; " 
                                "(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('" + text + "');\"";
        #elif __APPLE__
            std::string command = "say \"" + text + "\"";
        #else  
            std::string command = "espeak \"" + text + "\"";
        #endif
            std::system(command.c_str());
    }
}