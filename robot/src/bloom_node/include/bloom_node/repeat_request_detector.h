#ifndef BLOOM_NODE_REPEAT_REQUEST_DETECTOR_H
#define BLOOM_NODE_REPEAT_REQUEST_DETECTOR_H

// Phrase matcher for "please repeat that" style student speech.
//
// This is a C++ port of backend/Services/RepeatRequestDetector.cs and must be
// kept in sync with it. It lives on the robot because the check has to run
// *before* LessonCoordinator grades a response: by the time the backend sees
// the logged utterance, the robot has already scored "can you repeat that?"
// as an incorrect answer, spoken the incorrect-response script and possibly
// advanced to the next step.

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace bloom_node {
namespace repeat_request {

// Lowercase, replace anything outside [a-z0-9 whitespace] with a space,
// collapse whitespace, trim. Same normalization as the backend.
inline std::string normalize(const std::string &text) {
    std::string out;
    out.reserve(text.size());
    bool pending_space = false;
    for (unsigned char c : text) {
        char lc = static_cast<char>(std::tolower(c));
        bool keep = (lc >= 'a' && lc <= 'z') || (lc >= '0' && lc <= '9');
        if (keep) {
            if (pending_space && !out.empty()) out.push_back(' ');
            pending_space = false;
            out.push_back(lc);
        } else {
            pending_space = true;
        }
    }
    return out;
}

inline const std::vector<std::string> &phrases() {
    static const std::vector<std::string> list = [] {
        std::vector<std::string> raw = {
            "repeat that", "repeat it", "repeat what", "repeat please", "please repeat",
            "say that again", "say it again", "say again", "one more time",
            "what did you say", "what did you just say",
            "didn't hear", "did not hear", "didn't catch", "did not catch",
            "come again", "what was that",
            "read that again", "read it again", "do that again", "do it again",
            "play that again", "start over", "repeat",
            "could you repeat", "could you say that again", "could you say it again",
            "could you repeat that", "could you repeat it",
            "could you repeat what you just said",
        };
        for (auto &p : raw) p = normalize(p);
        return raw;
    }();
    return list;
}

// Short utterances only trusted as a repeat request when they make up the
// whole (short) thing the student said.
inline bool is_standalone_word(const std::string &w) {
    static const std::unordered_set<std::string> words = {
        "repeat", "again", "huh", "what", "pardon", "sorry",
    };
    return words.count(w) > 0;
}

constexpr size_t kStandaloneWordLimit = 3;

inline bool is_repeat_request(const std::string &text) {
    std::string normalized = normalize(text);
    if (normalized.empty()) return false;

    for (const auto &phrase : phrases()) {
        if (normalized.find(phrase) != std::string::npos) return true;
    }

    std::istringstream iss(normalized);
    std::vector<std::string> words;
    for (std::string w; iss >> w;) words.push_back(w);
    return !words.empty() && words.size() <= kStandaloneWordLimit &&
           std::all_of(words.begin(), words.end(), is_standalone_word);
}

}  // namespace repeat_request
}  // namespace bloom_node

#endif  // BLOOM_NODE_REPEAT_REQUEST_DETECTOR_H
