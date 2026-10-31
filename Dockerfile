# Node 20 matches CI (ci.yml), and some frontend dependencies require it.
FROM docker.io/library/node:20 AS react-build

WORKDIR /app

# Install from the lockfile, as CI does: `npm install` without package-lock.json re-resolves the whole
# dependency tree from the registry on every build, which is slow and not reproducible. The cache mount
# keeps downloaded packages between builds on the same machine; the layer itself is cached until
# package.json or the lockfile change.
COPY frontend/package.json frontend/package-lock.json ./frontend/

WORKDIR /app/frontend
RUN --mount=type=cache,target=/root/.npm \
    npm ci --no-audit --no-fund

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
# The app serves the SPA from wwwroot (UseStaticFiles + MapFallbackToFile). dotnet publish only puts
# the React build there from a full checkout, which this build context is not, so copy it in.
COPY --from=react-build /app/frontend/build ./wwwroot


EXPOSE 5000
ENTRYPOINT ["dotnet", "bloom.dll"]
