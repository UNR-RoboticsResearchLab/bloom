FROM node:18 AS react-build

WORKDIR /app

# Copy React app source files
COPY frontend/package.json ./frontend/package.json

WORKDIR /app/frontend
RUN npm install

COPY frontend/ ./
RUN npm run build


# backend
FROM --platform=$BUILDPLATFORM mcr.microsoft.com/dotnet/sdk:9.0 AS build

# Install EF tools (optional, if running migrations in container)
ENV PATH="${PATH}:/root/.dotnet/tools"

WORKDIR /app
COPY backend/*.csproj ./
RUN dotnet restore

COPY backend/ ./
RUN mkdir -p /var/dpkeysf
RUN dotnet publish -c Release -o out

# Run migrations if needed - it broke on docker compose build because it requires a database connection
# RUN dotnet ef database update

FROM --platform=$BUILDPLATFORM mcr.microsoft.com/dotnet/sdk:9.0 AS deploy

RUN dotnet tool install -g dotnet-ef

WORKDIR /app
COPY --from=build /app/out .
COPY --from=react-build /app/frontend/build ./frontend/build


EXPOSE 5000
ENTRYPOINT ["dotnet", "bloom.dll"]
