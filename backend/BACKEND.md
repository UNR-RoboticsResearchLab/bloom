# Bloom Backend Architecture

## Overview

Bloom is an ASP.NET Core web application that manages robot sessions, user accounts, and real-time robot state monitoring. The backend is built with a service-oriented architecture using dependency injection and follows RESTful API conventions.

---

## Core Infrastructure

### Program.cs Configuration

The application is configured with the following key components:

#### Database Setup

- **Database**: MySQL 11.7.2
- **ORM**: Entity Framework Core
- **Connection Management**:
  - Supports both Development and Production connection strings
  - Automatic retry on failure (up to 5 retries with 30-second delays)
- **Data Protection**: Keys persisted to `/var/dpkeys` with 90-day lifetime

#### Authentication & Authorization

- **Scheme**: Cookie-based authentication
- **Identity**: ASP.NET Core Identity with custom Account model
- **Password Requirements**:
  - Minimum 6 characters
  - At least one digit
  - Non-alphanumeric characters not required
- **Session Management**: 30-minute idle timeout with HttpOnly cookies

#### Services Registered

- **Account Service**: `IAccountService` -> `AccountService` (Scoped)
- **Robot Service**: `IRobotService` -> `RobotService` (Scoped)
- **Robot Session Service**: `IRobotSessionService` -> `RobotSessionService` (Scoped)
- **Robot State Service**: `IRobotStateService` -> `RobotStateService` (Scoped)
- **Session Code Service**: `ISessionCodeService` -> `SessionCodeService` (Scoped)
- **Robot State Repository**: `IRobotStateRepository` -> `InMemoryRobotStateRepository` (Singleton)
- **Robot Session Repository**: `IRobotSessionRepository` -> `RobotSessionRepository` (Scoped)

#### Middleware Pipeline

- CORS: Enabled for all origins, headers, and methods (development)
- Static Files: Serves default files and static assets
- Authentication: Validates user identity via cookies
- Authorization: Enforces access control policies
- Routing: Maps all controllers automatically

#### Development Features

- Swagger UI and OpenAPI documentation
- HTTPS configuration (commented for development)

---

## API Controllers

### 1. AccountsController

**Route**: `/api/accounts`

Handles user authentication, registration, and profile management.

#### Endpoints

| Method | Route | Description |
| -------- | ------- | ------------- |
| POST | `/login` | Authenticate user and create session |
| POST | `/create` | Register a new user account |
| GET | `/{id}` | Retrieve user profile (Authorized) |
| DELETE | `/{id}` | Delete user account (Authorized) |

#### Key Features

- **Role-Based Registration**: Supports 5 roles - Admin, Facilitator, Student, Teacher, SLP (planned transition to Admin, Educator (clinitian), Student)
- **Cookie Authentication**: Signs users in via claims-based identity
- **Profile Management**: Includes registered robots, assignments, and lessons

#### Response Structure

```json
{
  "Message": "Login successful",
  "User": {
    "Id": "string",
    "UserName": "string",
    "Email": "string",
    "FullName": "string",
    "EmailConfirmed": "boolean",
    "Role": "string"
  }
}
```

---

### 2. RobotsController

**Route**: `/api/robots`

Manages robot registration, updates, and queries.

#### Endpoints

| Method | Route | Description |
| -------- | ------- | ------------- |
| POST | `/register` | Register a new robot |
| GET | `` | Get all robots |
| GET | `/{id}` | Get specific robot by ID |
| PUT | `/{id}` | Update robot configuration |
| DELETE | `/{id}` | Delete a robot |
| GET | `/user/{userId}` | Get robots owned by user |
| GET | `/firmware/{firmwareVersion}` | Get robots by firmware version |

#### Key Features

- **Robot Registration**: Creates new robot entries with metadata
- **Firmware Tracking**: Supports querying by firmware version
- **User Association**: Links robots to specific users

---

### 3. RobotSessionsController

**Route**: `/api/robotsessions`

Manages active robot sessions, state tracking, and historical data.

#### Endpoints

| Method | Route | Description |
|--------|-------|-------------|
| GET | `` | Get all sessions (newest first) |
| POST | `` | Start a new session |
| GET | `/{sessionId}` | Get session details |
| POST | `/{sessionId}/end` | End an active session |
| GET | `/{sessionId}/robots` | Get robots in session |
| POST | `/{sessionId}/robots` | Add robot to session |
| DELETE | `/{sessionId}/robots/{robotId}` | Remove robot from session |
| GET | `/{sessionId}/states` | Get current robot states |
| PUT | `/{sessionId}/robots/{robotId}/state` | Update robot state |
| GET | `/{sessionId}/history` | Get session history |

#### Key Features

- **Session Management**: Create, retrieve, and end sessions
- **Anonymous Sessions**: Support for unauthenticated users
- **Real-Time State Tracking**: Monitor current robot status
- **Historical Data**: Maintains timestamped state snapshots
- **Ownership Verification**: Authenticated users can only access their sessions
- **Multi-Robot Support**: Sessions can contain multiple robots

#### Session States Include

- `Status`: Current operational state
- `CurrentTask`: Active task identifier
- `CurrentBehaviorId`: Active behavior identifier
- `SpeechLog`: Robot speech history
- `LastStatusChange`: Timestamp of last update

#### Response Structure

```json
{
  "Id": "uuid",
  "UserId": "string|null",
  "CreatedAt": "datetime",
  "LastUpdatedAt": "datetime",
  "Robots": ["uuid1", "uuid2"]
}
```

---

## Data Flow

### Session Lifecycle

1. **Create Session**: User initiates session (authenticated or anonymous)
2. **Add Robots**: Associate robots with the session
3. **Update States**: Track real-time robot state changes
4. **Archive History**: State changes are recorded for analysis
5. **End Session**: Close session and preserve historical data

### State Management

- **Current State**: Maintained in-memory via `InMemoryRobotStateRepository` (Singleton)
- **Historical State**: Persisted to database via `RobotSessionRepository` (Scoped)
- **Snapshot Pattern**: Each state update creates a new timestamped record

---

## Error Handling

All controllers implement consistent error handling:

| Status | Scenario |
| -------- | ---------- |
| 400 | Invalid model state, missing parameters, bad requests |
| 401 | User authentication failed or required |
| 403 | User lacks permission (ownership verification failed) |
| 404 | Resource not found |
| 500 | Server error with detailed logging |

---

## Security Features

- **Cookie-Based Sessions**: HttpOnly, secure cookies prevent XSS attacks
- **Ownership Verification**: Sessions can only be modified by their owner
- **Role-Based Access**: Different user roles with different capabilities
- **Claim-Based Identity**: Uses ClaimTypes.NameIdentifier for user identification
- **Request Validation**: ModelState validation on all endpoints

---

## Dependencies

- **Microsoft.AspNetCore.Identity**: User management and authentication
- **Microsoft.EntityFrameworkCore**: Database ORM
- **Pomelo.EntityFrameworkCore.MySql**: MySQL database provider
- **Microsoft.AspNetCore.Authentication.Cookies**: Cookie authentication scheme
- **Microsoft.AspNetCore.DataProtection**: Secure key management

---

## Configuration

### appsettings.json

- Connection strings for Development and Production environments
- Login/Logout paths for authentication redirects
- Kestrel server configuration (optional)

### Environment-Specific

- **Development**: Swagger UI enabled, CORS permissive
- **Production**: HSTS enabled, secure defaults applied
