# Interface for STT engines so we can swap between cloud and local models if needed
# Example: if we need lower latency or offline capability, we could switch to a local Whisper model

from abc import ABC, abstractmethod
from typing import Optional

class TranscriptionMetrics:
    def __init__(
        self,
        total_latency_ms: float,
        audio_duration_ms: Optional[float],
        success: bool,
        error_reason: Optional[str]
    ):
        self.total_latency_ms = total_latency_ms
        self.audio_duration_ms = audio_duration_ms
        self.success = success
        self.error_reason = error_reason

class TranscriptionResult:
    """
    text: The transcribed text (or None if transcription failed)
    metrics: TranscriptionMetrics object containing performance and status data
    """
    def __init__(
        self,
        text: Optional[str],
        metrics: TranscriptionMetrics
    ):
        self.text = text
        self.metrics = metrics

class STTEngineInterface(ABC):
    """
    Transcribe audio from a file
    Args:
        audio_filepath: Path to the audio file to transcribe
        language: Optional language code (e.g., 'en' for English)
    Returns:
        TranscriptionResult object containing:
          - text: transcribed text (or None if failed)
          - metrics: TranscriptionMetrics with latency, success/failure, etc
    """
    @abstractmethod
    def transcribe_file(
        self,
        audio_filepath: str,
        language: Optional[str] = None
    ) -> TranscriptionResult:
        pass

    """
    Record audio from microphone and transcribe it
    Args:
        duration_seconds: How long to record (default 5 seconds)
        language: Optional language code (e.g., 'en' for English)
    Returns:
        TranscriptionResult object containing:
          - text: transcribed text (or None if failed)
          - metrics: TranscriptionMetrics with latency, success/failure, etc
    """
    @abstractmethod
    def transcribe_from_mic(
        self,
        duration_seconds: float = 5.0,
        language: Optional[str] = None
    ) -> TranscriptionResult:
        pass