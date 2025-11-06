#If there is too much latency in the system, this interface can be used to swap TTS model to a PI based model

from abc import ABC, abstractmethod
from typing import List, Optional

class VisemeEvent:
    def __init__(self, timestamp_ms: int, viseme_id: str):
        self.timestamp_ms = timestamp_ms
        self.viseme_id = viseme_id

    def __repr__(self):
        return f"VisemeEvent(timestamp_ms={self.timestamp_ms}, viseme_id='{self.viseme_id}')"

class SynthesisMetrics:
    def __init__(
        self,
        total_latency_ms: float,
        first_byte_latency_ms: Optional[float],
        network_latency_ms: Optional[float],
        num_viseme_events: int,
        success: bool,
        error_reason: Optional[str]
    ):
        self.total_latency_ms = total_latency_ms
        self.first_byte_latency_ms = first_byte_latency_ms
        self.network_latency_ms = network_latency_ms
        self.num_viseme_events = num_viseme_events
        self.success = success
        self.error_reason = error_reason

class SynthesisResult:
    def __init__(
        self,
        audio_filepath: Optional[str],
        viseme_events: List[VisemeEvent],
        metrics: SynthesisMetrics
    ):
        """
        audio_filepath: Path to the generated audio file (or None if synthesis failed)
        viseme_events: List of VisemeEvent objects (with timestamp_ms and viseme_id)
        metrics: SynthesisMetrics object containing performance and status data
        """
        self.audio_filepath = audio_filepath
        self.viseme_events = viseme_events
        self.metrics = metrics

class TTSEngineInterface(ABC):
    @abstractmethod
    def synthesize(
        self,
        text: str,
        voice_id: Optional[str] = None,
        include_viseme: bool = True
    ) -> SynthesisResult:
        """
        Synthesize the given text into speech.
        Args:
            text: Text to synthesize.
            voice_id: Optional identifier of voice to use.
            include_viseme: Whether viseme data should be produced.
        Returns:
            SynthesisResult object containing:
              - audio_filepath: path to the generated audio file (or None if failed)
              - viseme_events: list of VisemeEvent objects
              - metrics: SynthesisMetrics object with latency, success/failure, etc.
        """
        pass
