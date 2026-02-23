


namespace bloom.Models
{

    public class LessonInteraction
    {

        public Guid Id { get; set; } = Guid.NewGuid();
        public Guid LessonId { get; set; }
        public Lesson? Lesson { get; set; }
        public Guid RobotSessionId { get; set; }
        public RobotSession? RobotSession { get; set; }

    }

}