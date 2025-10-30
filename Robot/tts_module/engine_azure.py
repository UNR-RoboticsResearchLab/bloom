import time
from typing import List, Optional

import azure.cognitiveservices.speech as speechsdk

from engine_interface import TTSEngineInterface, VisemeEvent, SynthesisResult

class SynthesisMetrics:
    def __init__(self,
                 total_latency_ms: float,
                 first_byte_latency_ms: Optional[float],
                 network_latency_ms: Optional[float],
                 num_viseme_events: int,
                 success: bool,
                 error_reason: Optional[str]):
        self.total_latency_ms = total_latency_ms
        self.first_byte_latency_ms = first_byte_latency_ms
        self.network_latency_ms = network_latency_ms
        self.num_viseme_events = num_viseme_events
        self.success = success
        self.error_reason = error_reason

class AzureTTSEngine(TTSEngineInterface):
    def __init__(self,
                 subscription_key: str, 
                 region: str,
                 default_voice: str = "en-US-AvaNeural"):
        self.subscription_key = subscription_key
        self.region = region
        self.default_voice = default_voice

        self.speech_config = speechsdk.SpeechConfig(
            subscription=self.subscription_key,
            region=self.region
        )
        self.speech_config.speech_synthesis_voice_name = self.default_voice

    def synthesize(self,
                   text: str,
                   voice_id: Optional[str] = None,
                   include_viseme: bool = True) -> SynthesisResult:
        
        t_start = time.time()

        voice = voice_id if voice_id else self.default_voice
        self.speech_config.speech_synthesis_voice_name = voice

        audio_filename = f"BlossomResponse.wav"
        audio_config = speechsdk.audio.AudioOutputConfig(filename=audio_filename)

        synthesizer = speechsdk.SpeechSynthesizer(
            speech_config=self.speech_config,
            audio_config=audio_config
        )

        viseme_events: List[VisemeEvent] = []
        if include_viseme:
            def viseme_callback(evt: speechsdk.SpeechSynthesisVisemeEventArgs):
                offset_ms = int(evt.audio_offset / 10000)
                viseme_events.append(VisemeEvent(timestamp_ms=offset_ms, viseme_id=str(evt.viseme_id)))
            synthesizer.viseme_received.connect(viseme_callback)


        result = synthesizer.speak_text_async(text).get()

        t_end = time.time()
        total_latency_ms = (t_end - t_start) * 1000

        #These metrics aren't available for all voices and regions so this try except statement handles that
        first_byte_latency_ms = None
        network_latency_ms = None
        try:
            props = result.properties
            first_byte_latency_ms = float(props.get_property(speechsdk.PropertyId.SpeechServiceResponse_SynthesisFirstByteLatencyMs))
            network_latency_ms = float(props.get_property(speechsdk.PropertyId.SpeechServiceResponse_SynthesisNetworkLatencyMs))
        except Exception:
            pass

        success = False
        error_reason: Optional[str] = None
        if result.reason == speechsdk.ResultReason.SynthesizingAudioCompleted:
            success = True
        else:
            cancellation = speechsdk.CancellationDetails.from_result(result)
            error_reason = f"{cancellation.reason}; details: {cancellation.error_details}"

        metrics = SynthesisMetrics(
            total_latency_ms=total_latency_ms,
            first_byte_latency_ms=first_byte_latency_ms,
            network_latency_ms=network_latency_ms,
            num_viseme_events=len(viseme_events),
            success=success,
            error_reason=error_reason
        )

        if not success:
            # If synthesis failed, return result with metrics but maybe without audio file
            return SynthesisResult(audio_filepath=None,
                                   viseme_events=[],
                                   metrics=metrics)

        return SynthesisResult(audio_filepath=audio_filename,
                               viseme_events=viseme_events,
                               metrics=metrics)