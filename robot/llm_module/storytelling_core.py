"""Scripted storytelling policy. No ROS, microphone, LLM or API dependency."""

from dataclasses import dataclass
import math
import re


VERSION = "storytelling-prototype-3"
CLOSING = (
    "Thanks for sharing your ideas with me! "
    "It was nice talking with you. Have a lovely day. Bye!"
)
CONFIRM_FINISH = "Have you finished telling me your ideas?"
CLARIFY = (
    "I mean one of the things you already told me. "
    "You can choose which one to tell me more about."
)
FINISH_CUES = {
    "the end", "that's the end", "that is the end", "that's all", "that is all",
    "that's my story", "that is my story", "i'm finished", "im finished",
    "i am finished", "i'm done", "im done", "i am done", "i want to stop",
    "can we stop", "stop please", "please stop", "i don't want to continue",
    "that will be it", "that's it", "that is it", "all done", "i'm all done",
    "i am all done", "nothing else", "that's everything", "that is everything",
    "i have finished", "i finished my story", "i'm finished now", "im finished now",
}


@dataclass(frozen=True)
class Settings:
    silence_seconds: float = 10.0
    thinking_seconds: float = 20.0
    max_followups: int = 2
    help_pause_seconds: float = 1.0

    def __post_init__(self):
        if any(not math.isfinite(value) or value <= 0 for value in
               (self.silence_seconds, self.thinking_seconds, self.help_pause_seconds)):
            raise ValueError("Silence and thinking allowances must be positive.")
        if not 0 <= self.max_followups <= 2:
            raise ValueError("This reviewed prototype supports zero, one or two follow-ups.")


@dataclass(frozen=True)
class Action:
    kind: str
    text: str
    reason: str


def normalized(text):
    text = text.lower().replace("’", "'")
    value = " ".join(re.sub(r"[^a-z0-9' ]", " ", text).split())
    value = re.sub(r"\b(i am|i'm|im)(?:\s+(?:i am|i'm|im))+\b", r"\1", value)
    # Speech recognizers can produce "I'm I'm finished" or "I I am done".
    value = re.sub(r"\b([a-z']+)(?:\s+\1\b)+", r"\1", value)
    return value


def control_intent(text):
    """Conservative standalone cues; not a semantic story-completeness model."""
    value = normalized(text)
    value = re.sub(r"^(?:(?:um+|uh+|hmm+|okay|ok|well|so|yes|yeah|yea|yep|i think|i guess)\s+)+", "", value)
    value = re.sub(r"\s+(?:thanks|thank you|please)$", "", value)
    if value in FINISH_CUES:
        return "finish"
    if value in {"like what idea", "what idea", "which idea", "what ideas",
                 "what do you mean", "what do you mean by that", "i don't understand",
                 "i dont understand", "can you explain", "can you explain that",
                 "what should i say", "what do i have to say"}:
        return "clarify"
    if value in {"i don't know", "i dont know", "i dunno", "i'm stuck", "im stuck",
                 "i am stuck", "can you help me", "help me please"}:
        return "help"
    if value in {"i'm thinking", "im thinking", "i am thinking", "let me think",
                 "wait i'm thinking", "wait im thinking", "i need to think"}:
        return "thinking"
    return "content"


def confirmation_response(text):
    """Interpret agreement only in reply to the robot's finish question."""
    value = normalized(text)
    if value in {"uh huh", "mm hmm", "mhm", "mmhm"}:
        return "finish"
    value = re.sub(r"^(?:(?:um+|uh+|hmm+|well|okay|ok|so)\s+)+", "", value)
    value = re.sub(r"\s+(?:thanks|thank you|please)$", "", value)
    # Continuing takes priority over an initial yes in 'yes, but I want to add...'.
    if value in {"no", "nope", "nah", "not yet", "no not yet", "not really",
                 "i don't think so", "i dont think so", "i guess not", "i think not",
                 "i need more time", "i'm still thinking", "im still thinking"} or re.search(
        r"\b(?:not (?:done|finished|yet)|still thinking|want to (?:add|continue|say)|"
        r"wanna (?:add|continue|say)|let me (?:add|think)|can i add|one more thing|need more time)\b", value
    ):
        return "continue"
    if control_intent(text) == "finish" or value in {
        "yes", "yeah", "yea", "yep", "yup", "yes i am", "yes i'm done", "yes im done",
        "sure", "i guess so", "guess so", "i think so", "think so", "i suppose so",
        "yes i guess so", "yeah i guess so", "yes i think so", "yeah i think so",
        "suppose so", "pretty much", "yes pretty much", "yeah pretty much",
        "that's about it", "that is about it", "that would be it", "that should be it",
        "that's all i have", "that is all i have", "that's all i can think of",
        "that is all i can think of", "i have nothing else", "nothing more",
        "i'm good", "im good", "i am good", "yes that's it", "yeah that's it",
    }:
        return "finish"
    return "uncertain"


def split_finish_cue(text):
    """Keep preceding content when an explicit closing cue is a final sentence."""
    if control_intent(text) == "finish":
        return "", True
    # Look only at an independent final sentence, not quoted story dialogue.
    parts = list(re.finditer(r"[.!?]\s+", text))
    if parts:
        last = parts[-1]
        tail = text[last.end():]
        if not any(mark in tail for mark in ('"', '“', '”')) and control_intent(tail) == "finish":
            return text[:last.start()].strip(), True
    match = re.search(
        r"(?:[.!?]\s+)(the end|that's (?:all|my story|the end)|"
        r"that is (?:all|my story|the end)|i(?:'m| am) (?:done|finished))[.!?\s]*$",
        text.replace("’", "'"), re.IGNORECASE,
    )
    if match:
        return text[:match.start()].strip(), True
    return text, False


def interval_union_seconds(intervals):
    """Merge recognized audio offsets, measured in seconds within one window."""
    merged = []
    for start, end in sorted(intervals):
        if start < 0 or end <= start:
            continue
        if merged and start <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], end)
        else:
            merged.append([start, end])
    return sum(end - start for start, end in merged)


class StorySession:
    def __init__(self, task, robot_name, settings, emit, now, assessor=None):
        self.task = task
        self.robot_name = robot_name
        self.settings = settings
        self.emit = emit
        self.started = now
        self.last_activity = now
        self.initial_prompt_end = None
        self.closing_start = None
        self.phase = "created"
        self.followups = 0
        self.content_segments = []
        self.pending_speech = False
        self.finish_requested = False
        self.help_requested = False
        self.thinking_until = None
        self.assessor = assessor
        self.revision = 0
        self.last_child_text = ""
        self.confirmations = 0
        self.clarifications = 0
        self.confirmation_pending = False
        self.confirmation_response_needs_check = False
        self.clarification_requested = False
        self.assessment_wait_until = None
        self.assessment_wait_revision = None
        self.assessment_logged_revision = None
        self.virtual_wait = False

    def intro(self):
        return Action(
            "intro",
            f"Hi! I'm {self.robot_name}. It's so nice to meet you! "
            "I have something for you to imagine. "
            f"{self.task['spoken_prompt']} "
            "Take a moment to think, and start when you're ready. "
            "When you're done, just say, 'I'm finished.'",
            "researcher_assigned_prompt",
        )

    def speaking(self, action, now):
        self.phase = "speaking"
        self.pending_speech = False
        self.help_requested = False
        self.thinking_until = None
        self.clarification_requested = False
        if action.kind == "support":
            self.followups += 1
        if action.kind == "closing":
            self.closing_start = now
        if action.kind == "confirmation":
            self.confirmations += 1
            self.confirmation_pending = True
            self.confirmation_response_needs_check = False
        if action.kind == "clarification":
            self.clarifications += 1
        self.emit("robot_utterance", now, kind=action.kind, text=action.text,
                  reason=action.reason, support_count=self.followups)

    def robot_finished(self, action, now):
        self.emit("robot_utterance_finished", now, kind=action.kind)
        if action.kind == "closing":
            self.phase = "finished"
            return
        if action.kind == "intro":
            self.initial_prompt_end = now
        self.phase = "listening"
        self.last_activity = now

    def speech_activity(self, now):
        if self.phase == "listening":
            self.pending_speech = True
            self.last_activity = now
            self.revision += 1

    def final_segment(self, text, now):
        if self.phase != "listening":
            return
        self.pending_speech = False
        self.last_activity = now
        text = text.strip()
        if not text:
            return
        self.revision += 1
        self.last_child_text = text
        content, finished = split_finish_cue(text)
        intent = control_intent(text)
        value = normalized(text)
        if self.confirmation_pending or self.confirmation_response_needs_check:
            self.confirmation_pending = False
            response = confirmation_response(text)
            if finished or response == "finish":
                content, finished, intent = "", True, "finish"
                self.confirmation_response_needs_check = False
            elif response == "continue":
                content, finished, intent = "", False, "thinking"
                self.confirmation_response_needs_check = False
            else:
                # Unfamiliar replies are interpreted against the actual robot
                # question, not sent back through generic story assessment.
                self.confirmation_response_needs_check = True
                content, intent = "", "confirmation_reply"
        elif self.followups > 0 and value in {"no", "nope", "no thank you", "nothing else"}:
            content, finished, intent = "", True, "finish"
        self.emit("child_segment", now, text=text, stage=self.followups,
                  input_intent="finish" if finished else intent)
        if content and intent == "content":
            self.content_segments.append({"text": content, "stage": self.followups})
        self.finish_requested = finished
        self.help_requested = intent == "help"
        self.clarification_requested = intent == "clarify"
        self.thinking_until = (
            now + self.settings.thinking_seconds if intent == "thinking" else None
        )

    def explicit_finish(self, now, reason="researcher_finish"):
        self.emit("finish_requested", now, reason=reason)
        self.finish_requested = True
        self.pending_speech = False

    def next_action(self, now):
        """Inspect, without consuming the quota; commit only before playback."""
        if self.phase != "listening" or self.pending_speech:
            return None
        if self.finish_requested:
            return Action("closing", CLOSING, "explicit_finish")
        if self.clarification_requested:
            if now - self.last_activity < self.settings.help_pause_seconds:
                return None
            if self.clarifications < 2:
                return Action("clarification", CLARIFY, "clarification_request")
            if self.confirmations == 0:
                return Action("confirmation", CONFIRM_FINISH, "repeated_clarification")
            return Action("closing", CLOSING, "clarification_limit_after_finish_check")
        if self.thinking_until is not None and now < self.thinking_until:
            return None
        allowance = (
            self.settings.help_pause_seconds if self.help_requested
            else self.settings.silence_seconds
        )
        if now - self.last_activity < allowance:
            return None
        if self.confirmation_pending:
            # The child was explicitly invited to confirm. Do not repeatedly
            # ask the same question if there is no response.
            return Action("closing", CLOSING, "no_response_to_completion_confirmation")
        if self.confirmation_response_needs_check:
            decision = "UNCERTAIN"
            if self.assessor is not None:
                decision = self.assessor.check(
                    self.revision, self.task, self.transcript(), self.last_child_text,
                    context={"awaiting_finish_confirmation": True,
                             "robot_last_question": CONFIRM_FINISH},
                )
                if decision is None:
                    return None
            self.emit("finish_reply_assessment", now, decision=decision,
                      revision=self.revision)
            if decision == "COMPLETE":
                return Action("closing", CLOSING, "accepted_finish_confirmation")
            if decision == "NEEDS_SUPPORT":
                self.content_segments.append({"text": self.last_child_text,
                                              "stage": self.followups,
                                              "from_confirmation_reply": True})
            # Do not re-ask or rephrase the finish question. If the answer is
            # unclear or signals more thought, silently leave time to continue.
            self.confirmation_response_needs_check = False
            self.thinking_until = now + self.settings.thinking_seconds
            return None
        if self.content_segments and not self.help_requested:
            decision = "NEEDS_SUPPORT"
            if self.assessor is not None:
                decision = self.assessor.check(
                    self.revision, self.task, self.transcript(), self.last_child_text,
                )
                if decision is None:
                    return None  # Recognition continues while the LLM checks.
                if self.assessment_logged_revision != self.revision:
                    self.emit("completion_assessment", now, decision=decision,
                              revision=self.revision,
                              diagnostic=getattr(self.assessor, "diagnostic", None))
                    self.assessment_logged_revision = self.revision
            if decision in ("COMPLETE", "UNCERTAIN"):
                if self.confirmations == 0:
                    return Action("confirmation", CONFIRM_FINISH, "check_possible_completion")
                return Action("closing", CLOSING, "finished_without_repeating_confirmation")
            if decision == "CLARIFY" and self.clarifications < 2:
                return Action("clarification", CLARIFY, "interpreted_clarification_request")
            if decision == "THINKING":
                if self.assessment_wait_revision != self.revision:
                    self.assessment_wait_revision = self.revision
                    self.assessment_wait_until = now + self.settings.thinking_seconds
                if not self.virtual_wait and now < self.assessment_wait_until:
                    return None
        if self.followups >= self.settings.max_followups:
            if self.confirmations == 0:
                return Action("confirmation", CONFIRM_FINISH, "support_limit_check_completion")
            return Action("closing", CLOSING, "completion_checks_then_silence")
        if not self.content_segments:
            if self.followups == 0:
                text = self.task["start_support"]
            else:
                text = "You can start with one idea. What would you like to say?"
            reason = "help_start"
        else:
            text = self.task["continuation_support"][self.followups]
            reason = "requested_help" if self.help_requested else "elaboration_after_silence"
        return Action("support", text, reason)

    def transcript(self):
        return " ".join(segment["text"] for segment in self.content_segments)