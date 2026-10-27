"""Exercise playback ordering without requiring ROS, pygame, or audio hardware."""
import ast
from pathlib import Path
import types
import unittest
from unittest.mock import Mock


class CompletionChimeTests(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).parents[1] / 'bloom_speech/rsr_speech_node.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RsrSpeechNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'play_command')
        self.events = Mock()
        self.music = Mock()
        self.music.get_busy.return_value = False
        self.events.attach_mock(self.music, 'music')
        self.pub = Mock()
        self.events.attach_mock(self.pub.publish, 'chime')
        namespace = {'pygame': types.SimpleNamespace(mixer=types.SimpleNamespace(music=self.music)),
                     'time': Mock(), 'os': Mock(), 'String': types.SimpleNamespace}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        self.play = namespace['play_command']
        self.node = types.SimpleNamespace(
            get_logger=Mock(), download_to_temp=Mock(return_value='/temporary/audio.wav'),
            download_visemes=Mock(return_value=[]), stop_audio_sync=Mock(),
            set_face_mode=Mock(), chime_pub=self.pub)

    def test_chime_follows_finished_playback(self):
        self.play(self.node, {'audioUrl': '/sentence.wav', 'visemeUrl': '/visemes.json'})
        names = [call[0] for call in self.events.mock_calls]
        self.assertLess(names.index('music.unload'), names.index('chime'))
        self.assertEqual('prompt_complete', self.pub.publish.call_args.args[0].data)

    def test_failed_playback_has_no_completion_chime(self):
        self.music.play.side_effect = RuntimeError('Audio device failed')
        with self.assertRaises(RuntimeError):
            self.play(self.node, {'audioUrl': '/sentence.wav', 'visemeUrl': '/visemes.json'})
        self.pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
