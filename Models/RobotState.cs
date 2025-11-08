
using Microsoft.EntityFrameworkCore;

namespace bloom.Models
{

    [Owned]
    public class RobotState
    {
        public Guid Id { get; set; } = Guid.NewGuid();
        public Guid RobotId { get; set; }

        /**
         * Status is the current robot status.
         *     waiting, speaking, loading, error
         */
        public required string Status { get; set; }

        /**
         * Current task is the current robot task.
         *     giving lesson, free speech, activity, etc.
         */
        public required string CurrentTask { get; set; }

        /**
         * Current behavior the robot is completing.
         *     breathing, nodding, excited, waiting, etc.
         */
        public int? CurrentBehaviorId { get; set; }
        public Behavior? CurrentBehavior { get; set; }

        /**
         * LastStatusChange is the last time the robot firmware status changed
         */
        public DateTime LastStatusChange { get; set; }

        /**
         * SpeechLog since last message
         */
        public string SpeechLog { get; set; } = "";
    }
}