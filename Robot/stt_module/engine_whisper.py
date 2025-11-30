import time
import os
from typing import Optional

import sounddevice as sd
import soundfile as sf
import numpy as np
from openai import OpenAI

from engine_interface import STTEngineInterface, TranscriptionResult, TranscriptionMetrics


"""
OpenAI Whisper API implementation for speech-to-text
Supports file-based transcription and microphone recording for Bloom robot
"""
class WhisperSTTEngine(STTEngineInterface):
    
    def __init__(self, api_key: str, model: str = "whisper-1", input_device: Optional[int] = None):
        self.api_key = api_key
        self.model = model
        self.client = OpenAI(api_key=self.api_key)
        
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
        
        # Audio recording settings
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
            
            # Open and send to Whisper API
            with open(audio_filepath, "rb") as audio_file:
                transcription = self.client.audio.transcriptions.create(
                    model=self.model,
                    file=audio_file,
                    language=language
                )
            
            t_end = time.time()
            total_latency_ms = (t_end - t_start) * 1000
            
            metrics = TranscriptionMetrics(
                total_latency_ms=total_latency_ms,
                audio_duration_ms=audio_duration_ms,
                success=True,
                error_reason=None
            )
            
            return TranscriptionResult(
                text=transcription.text,
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
            
            # Save to temporary WAV file
            temp_filepath = "temp_recording.wav"  # TODO: make this configurable later
            sf.write(temp_filepath, audio_data, self.sample_rate, subtype='PCM_16')
            
            # Check if we recorded anything
            max_vol = np.max(np.abs(audio_data))
            print(f"DEBUG: Max volume recorded: {max_vol:.4f}")
            if max_vol < 0.01:
                print("WARNING: Very quiet audio detected!")
            
            audio_duration_ms = duration_seconds * 1000
            
            # Send to Whisper API
            with open(temp_filepath, "rb") as audio_file:
                transcription = self.client.audio.transcriptions.create(
                    model=self.model,
                    file=audio_file,
                    language=language
                )
            
            t_end = time.time()
            total_latency_ms = (t_end - t_start) * 1000
            
            metrics = TranscriptionMetrics(
                total_latency_ms=total_latency_ms,
                audio_duration_ms=audio_duration_ms,
                success=True,
                error_reason=None
            )
            
            # Clean up temp file
            if temp_filepath and os.path.exists(temp_filepath):
                os.remove(temp_filepath)
            
            return TranscriptionResult(
                text=transcription.text,
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