

using bloom.Models;

namespace bloom.Services
{
    public interface IRobotStateService
    {

        /**
         * <summary> 
         * GetCurrentRobotStateByRobotIdAsync() gets the current state from the robot Id
         * </summary>
         * <returns>RobotState</returns>
        */
        RobotState GetCurrentRobotStateByRobotIdAsync(string robotId);

        /**
         * <summary>
         * GetCurrentRobotStateBySessionIdAsync() gets all current states from the session Id
         * </summary>
         * <returns>Collection of RobotStates</returns>
        */
        ICollection<RobotState> GetCurrentRobotStateBySessionIdAsync(string sessionId);

        /**
         * <summary>
         * GetCurrentRobotStatesByClassroomIdAsync() gets the current robotstates by a classroomId
         * </summary>
         */
        ICollection<RobotState> GetCurrentRobotStatesByClassroomIdAsync(string classroomId);

        /**
         * <summary>
         * GetRobotStateBySessionId() gets all of the robots states with a specific sessionId
         * </summary>
         * <returns>List of RobotStates</returns>
         */
        ICollection<RobotState> GetRobotStatesBySessionId(string sessionId);

        /**
         * <summary>
         * GetAllCurrentRobotStatesAsync() gets the states all of the robots currently registered in the system.
         * </summary>
         * <returns>List of RobotStates</returns>
        */
        ICollection<RobotState> GetAllCurrentRobotStatesAsync();

    }
}