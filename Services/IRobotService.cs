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
        // Get robot state
        Task<RobotState> GetRobotStateAsync(string robotId);
        
        // Update robot state
        Task UpdateRobotStateAsync(string robotId, RobotState state);
        
        // Get current assignment
        Task<Assignment> GetCurrentAssignmentAsync(string robotId);
        
        // Assign task to robot
        Task AssignTaskAsync(string robotId, Assignment assignment);
        
        // Complete or update assignment status
        Task UpdateAssignmentStatusAsync(string robotId, string assignmentId, Assignment status);
        
        // Get available robots
        Task<IEnumerable<string>> GetAvailableRobotsAsync();
        
        // Sync robot state with backend
        Task SyncRobotStateAsync(string robotId);
        
        // Register new robot
        Task RegisterRobotAsync(string robotId, RobotConfigDto config);
    }
}