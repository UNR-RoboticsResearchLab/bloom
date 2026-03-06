## This is a document I'm using to come up with a json structure for lessons on the blossom

```
ros2 topic pub --once /load_lesson std_msgs/msg/String "data: '{\"id\":\"testlesson\",\"title\":\"Hello World\",\"learning_objectives\":[],\"sequence\":[{\"id\":1,\"type\":\"introduction\",\"script\":\"Hello\",\"behaviors\":{\"behavior\":\"listening\"},\"timing_seconds\":3}]}'"
```

```
ros2 topic pub --once /load_lesson std_msgs/msg/String \
  "data: '{\"id\":\"ex2\",\"title\":\"Three Steps\",\"learning_objectives\":[],\"sequence\":[{\"id\":1,\"type\":\"introduction\",\"script\":\"Welcome to lesson two\",\"behaviors\":{\"behavior\":\"happy\",\"facial_expression\":\"smile\"},\"timing_seconds\":3},{\"id\":2,\"type\":\"definition\",\"script\":\"This is step two\",\"behaviors\":{\"behavior\":\"idle\"},\"timing_seconds\":3},{\"id\":3,\"type\":\"closing\",\"script\":\"And that was the last step\",\"behaviors\":{\"behavior\":\"happy\"},\"timing_seconds\":2}]}'"
```

```json
{
  "id": "lesson_001",
  "lesson": {
    "title": "lesson_title",
    "grade": "lesson_grade",
    "duration": 10,
    "objectives": ["distinguish between homophones"]
  },
  "sequence": [
    {
      "id": 1,
      "type": "introduction",
      "script": "Hello, there! Today we're going to learn about homophones... Do you know what a homophone is?",
      "behaviors": {
        "behavior": "happy",
        "facial_expression": "smile",
        "gaze": "scan_room"
      },
      "timing_seconds": 10
    },
    {
      "id": 2,
      "type": "definition",
      "script": "Homophones are words that sound the same when we say them, but they have different meanings and different spellings.",
      "behaviors": {
        "behavior": "idle",
        "facial_expression": "smile",
        "gaze": "center"
      },
      "visual_aid": ["slide_definition_homophones.png"],
      "timing_seconds": 20
    },
    {
      "id": 3,
      "type": "example",
      "script": "For example: 'sea' and 'see'. They sound the same. But 'sea' is the ocean, and 'see' means to look at something.",
      "behaviors": {
        "behavior": "idle",
        "facial_expression": "engaged",
        "gaze": "left_to_right"
      },
      "visual_aid": "slide_sea_see.png",
      "timing_seconds": 25
    },
    {
      "id": 4,
      "type": "example",
      "script": "Another example is 'their', 'there', and 'they're'. These are  tricky! 'Their' shows ownership. 'There' tells a place. 'They're' means  they are.",
      "behaviors": {
        "behavior": "idle",
        "facial_expression": "thoughtful",
        "gaze": "center"
      },
      "visual_aid": "slide_their_there_theyre.png",
      "timing_seconds": 30
    },
    {
      "id": 5,
      "type": "interactive_question",
      "script": "Can anyone give me a pair of homophones?",
      "behaviors": {
        "behavior": "invite_response",
        "posture": "lean_forward",
        "gaze": "audience_member"
      },
      "interaction": {
        "wait_for_response": true,
        "max_wait_seconds": 10,
        "fallback_script": "One example is 'right' and 'write'."
      }
    },
    {
      "id": 6,
      "type": "guided_practice",
      "script": "Listen carefully. Which word fits the sentence: I will ___     you a letter. Is it 'write' or 'right'?",
      "behaviors": {
        "behaivior": "hand_to_ear",
        "facial_expression": "curious",
        "gaze": "center"
      },
      "interaction": {
        "wait_for_response": true,
        "max_wait_seconds": 8,
        "correct_answer": "write",
        "correct_response_script": "Correct! 'Write' means to put words on  paper.",
        "incorrect_response_script": "Not quite. 'Right' means correct or a     direction. 'Write' means to put words on paper."
      },
      "timing_seconds": 20
    },
    {
      "id": 7,
      "type": "explanation",
      "script": "To figure out which homophone to use, you need to look at the  meaning of the sentence. Context clues help us decide.",
      "behaviors": {
        "behavior": "tap_head_think",
        "facial_expression": "thoughtful",
        "gaze": "center"
      },
      "timing_seconds": 20
    },
    {
      "id": 8,
      "type": "activity",
      "script": "Let's say this together: 'I see the sea.' Notice how the   words sound the same but mean different things!",
      "behaviors": {
        "behavior": "encourage_repeat",
        "facial_expression": "playful",
        "gaze": "center"
      },
      "interaction": {
        "repeat_count": 2,
        "pause_between_repeats_seconds": 3
      },
      "timing_seconds": 20
    },
    {
      "id": 9,
      "type": "summary",
      "script": "Today we learned that homophones sound the same but have   different meanings and spellings. We use context to choose the correct    word.",
      "behaviors": {
        "behavior": "summarize",
        "facial_expression": "warm_smile",
        "gaze": "scan_room"
      },
      "timing_seconds": 15
    },
    {
      "id": 10,
      "type": "closing",
      "script": "Great job today! Keep listening carefully to words that sound  the same. Do you have any questions?",
      "behaviors": {
        "behavior": "",
        "posture": "relaxed",
        "gaze": "audience"
      },
      "interaction": {
        "wait_for_questions": true,
        "max_wait_seconds": 15
      }
    }
  ]
}
```
