// bloom
// IRobotService.cs
// Interface defining robot service operations to manage robot states and needed functionalities
// Created: 11/1/2025

using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using bloom.Models;

namespace bloom.Services
{
    public interface IRobotService
    {
        /**
         * <summary> 
         * GetRobotByUserIdAsync() gets all of the robots currently associated with a given user.
         * </summary>
         * <returns>List of robots</returns>
        */
        ICollection<Robot> GetRobotsByUserIdAsync(string userId);

        /**
         * <summary>
         * GetRobotsByFirmwareVersion() gets all of the robots with a specific firmware version.
         * <summary>
         * <returns>List of Robots</return>
         */
        ICollection<Robot> GetRobotsByFirmwareVersion(string firmwareVersion);

        /**
         * <summary>
         * GetAllRobotsAsync() gets all of the robots currently registered in the system.
         * </summary>
         * <returns>List of robots</returns>
        */
        ICollection<Robot> GetAllRobotsAsync();

        /**
         * <summary>
         * RegisterRobotAsync() registers a new robot in the system. Should be triggered on first boot.
         * </summary>
         * <returns>bool</returns>
         */
        bool RegisterRobotAsync(RobotDto robot);


    }
}