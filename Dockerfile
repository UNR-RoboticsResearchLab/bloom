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

# No .git in the build context, so the version is computed outside (nbgv get-version -v SemVer2)
# and passed in. Unset means a local build, stamped 0.0.0-local so it's never mistaken for a release.
ARG BUILD_VERSION=0.0.0-local

WORKDIR /app
COPY backend/*.csproj ./
RUN dotnet restore -p:BuildVersion=${BUILD_VERSION}

COPY backend/ ./
RUN mkdir -p /var/dpkeysf
RUN dotnet publish -c Release -o out -p:BuildVersion=${BUILD_VERSION}

# Run migrations if needed - it broke on docker compose build because it requires a database connection
# RUN dotnet ef database update

FROM --platform=$BUILDPLATFORM mcr.microsoft.com/dotnet/sdk:9.0 AS deploy

RUN dotnet tool install -g dotnet-ef

WORKDIR /app
COPY --from=build /app/out .
COPY --from=react-build /app/frontend/build ./frontend/build


EXPOSE 5000
ENTRYPOINT ["dotnet", "bloom.dll"]
