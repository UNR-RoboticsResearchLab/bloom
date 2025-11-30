import time
import os
from typing import Optional

import azure.cognitiveservices.speech as speechsdk
import sounddevice as sd
import soundfile as sf
import numpy as np

from engine_interface import STTEngineInterface, TranscriptionResult, TranscriptionMetrics


"""
Azure Speech-to-Text implementation for the Bloom robot
Supports file-based transcription and microphone recording
"""
class AzureSTTEngine(STTEngineInterface):
    
    def __init__(self, subscription_key: str, region: str, input_device: Optional[int] = None):
        self.subscription_key = subscription_key
        self.region = region
        
        # If no device specified, try to auto-detect or load from config
        if input_device is None:
            try:
                from config import MICROPHONE_DEVICE
                self.input_device = MICROPHONE_DEVICE
            except ImportError:
                # Config doesn't have MICROPHONE_DEVICE, use system default
                default_device = sd.query_devices(kind='input')
                self.input_device = default_device['index'] if isinstance(default_device, dict) else None
        else:
            self.input_device = input_device
        
        self.speech_config = speechsdk.SpeechConfig(
            subscription=self.subscription_key,
            region=self.region
        )
        
        # Audio recording settings for microphone mode
        self.sample_rate = 16000
        self.channels = 1
    
    """
    Transcribe an existing audio file
    """
    def transcribe_file(
        self,
        audio_filepath: str,
        language: Optional[str] = None
    ) -> TranscriptionResult:
        
        t_start = time.time()
        
        try:
            # Get audio duration for metrics
            audio_duration_ms = self._get_audio_duration_ms(audio_filepath)
            
            # Set language if provided
            if language:
                self.speech_config.speech_recognition_language = language
            else:
                self.speech_config.speech_recognition_language = "en-US"
            
            # Configure audio input from file
            audio_config = speechsdk.AudioConfig(filename=audio_filepath)
            
            # Create recognizer
            recognizer = speechsdk.SpeechRecognizer(
                speech_config=self.speech_config,
                audio_config=audio_config
            )
            
            # Perform recognition
            result = recognizer.recognize_once()
            
            t_end = time.time()
            total_latency_ms = (t_end - t_start) * 1000
            
            # Check result
            if result.reason == speechsdk.ResultReason.RecognizedSpeech:
                metrics = TranscriptionMetrics(
                    total_latency_ms=total_latency_ms,
                    audio_duration_ms=audio_duration_ms,
                    success=True,
                    error_reason=None
                )
                
                return TranscriptionResult(
                    text=result.text,
                    metrics=metrics
                )
            elif result.reason == speechsdk.ResultReason.NoMatch:
                # No speech detected
                metrics = TranscriptionMetrics(
                    total_latency_ms=total_latency_ms,
                    audio_duration_ms=audio_duration_ms,
                    success=False,
                    error_reason="No speech detected in audio"
                )
                
                return TranscriptionResult(
                    text=None,
                    metrics=metrics
                )
            elif result.reason == speechsdk.ResultReason.Canceled:
                # Recognition was cancelled
                cancellation = speechsdk.CancellationDetails.from_result(result)
                error_msg = f"{cancellation.reason}; details: {cancellation.error_details}"
                
                metrics = TranscriptionMetrics(
                    total_latency_ms=total_latency_ms,
                    audio_duration_ms=audio_duration_ms,
                    success=False,
                    error_reason=error_msg
                )
                
                return TranscriptionResult(
                    text=None,
                    metrics=metrics
                )
            else:
                # Unknown reason
                metrics = TranscriptionMetrics(
                    total_latency_ms=total_latency_ms,
                    audio_duration_ms=audio_duration_ms,
                    success=False,
                    error_reason=f"Recognition failed with reason: {result.reason}"
                )
                
                return TranscriptionResult(
                    text=None,
                    metrics=metrics
                )
            
        except Exception as e:
            t_end = time.time()
            total_latency_ms = (t_end - t_start) * 1000
            
            metrics = TranscriptionMetrics(
                total_latency_ms=total_latency_ms,
                audio_duration_ms=None,
                success=False,
                error_reason=str(e)
            )
            
            return TranscriptionResult(
                text=None,
                metrics=metrics
            )
    
    """
    Record from microphone and transcribe
    """
    def transcribe_from_mic(
        self,
        duration_seconds: float = 5.0,
        language: Optional[str] = None
    ) -> TranscriptionResult:
        
        t_start = time.time()
        temp_filepath = None
        
        try:
            # Record audio from microphone
            print(f"Recording for {duration_seconds} seconds...")
            audio_data = sd.rec(
                int(duration_seconds * self.sample_rate),
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype='float32',
                device=self.input_device
            )
            sd.wait()  # Wait until recording is finished
            print("Recording finished")
            
            # Save to temporary WAV file in 16-bit PCM format (required by Azure)
            temp_filepath = "temp_recording.wav"  # TODO: make this configurable later
            sf.write(temp_filepath, audio_data, self.sample_rate, subtype='PCM_16')

            # Check if we recorded anything
            max_vol = np.max(np.abs(audio_data))
            print(f"DEBUG: Max volume recorded: {max_vol:.4f}")
            if max_vol < 0.01:
                print("WARNING: Very quiet audio detected!")
            
            audio_duration_ms = duration_seconds * 1000
            
            # Set language if provided
            if language:
                self.speech_config.speech_recognition_language = language
            else:
                self.speech_config.speech_recognition_language = "en-US"
            
            # Configure audio input from temp file
            audio_config = speechsdk.AudioConfig(filename=temp_filepath)
            
            # Create recognizer
            recognizer = speechsdk.SpeechRecognizer(
                speech_config=self.speech_config,
                audio_config=audio_config
            )
            
            # Perform recognition
            result = recognizer.recognize_once()
            
            t_end = time.time()
            total_latency_ms = (t_end - t_start) * 1000
            
            # Clean up temp file
            if temp_filepath and os.path.exists(temp_filepath):
                os.remove(temp_filepath)
            
            # Check result
            if result.reason == speechsdk.ResultReason.RecognizedSpeech:
                metrics = TranscriptionMetrics(
                    total_latency_ms=total_latency_ms,
                    audio_duration_ms=audio_duration_ms,
                    success=True,
                    error_reason=None
                )
                
                return TranscriptionResult(
                    text=result.text,
                    metrics=metrics
                )
            elif result.reason == speechsdk.ResultReason.NoMatch:
                # No speech detected
                metrics = TranscriptionMetrics(
                    total_latency_ms=total_latency_ms,
                    audio_duration_ms=audio_duration_ms,
                    success=False,
                    error_reason="No speech detected in audio"
                )
                
                return TranscriptionResult(
                    text=None,
                    metrics=metrics
                )
            elif result.reason == speechsdk.ResultReason.Canceled:
                # Recognition was cancelled
                cancellation = speechsdk.CancellationDetails.from_result(result)
                error_msg = f"{cancellation.reason}; details: {cancellation.error_details}"
                
                metrics = TranscriptionMetrics(
                    total_latency_ms=total_latency_ms,
                    audio_duration_ms=audio_duration_ms,
                    success=False,
                    error_reason=error_msg
                )
                
                return TranscriptionResult(
                    text=None,
                    metrics=metrics
                )
            else:
                # Unknown reason
                metrics = TranscriptionMetrics(
                    total_latency_ms=total_latency_ms,
                    audio_duration_ms=audio_duration_ms,
                    success=False,
                    error_reason=f"Recognition failed with reason: {result.reason}"
                )
                
                return TranscriptionResult(
                    text=None,
                    metrics=metrics
                )
            
        except Exception as e:
            t_end = time.time()
            total_latency_ms = (t_end - t_start) * 1000
            
            # Clean up temp file if it exists
            if temp_filepath and os.path.exists(temp_filepath):
                os.remove(temp_filepath)
            
            metrics = TranscriptionMetrics(
                total_latency_ms=total_latency_ms,
                audio_duration_ms=None,
                success=False,
                error_reason=str(e)
            )
            
            return TranscriptionResult(
                text=None,
                metrics=metrics
            )
    
    """
    Helper to get audio file duration in milliseconds
    """
    def _get_audio_duration_ms(self, audio_filepath: str) -> Optional[float]:
        
        try:
            info = sf.info(audio_filepath)
            return (info.frames / info.samplerate) * 1000
        except Exception:
            return None