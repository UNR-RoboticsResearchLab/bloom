FROM node:18 AS react-build

WORKDIR /app

# Copy React app source files
COPY ClientApp/package.json ./ClientApp/package.json

WORKDIR /app/ClientApp
RUN npm install

COPY ClientApp/ ./
RUN npm run build


# backend
FROM --platform=$BUILDPLATFORM mcr.microsoft.com/dotnet/sdk:9.0 AS build

# Install EF tools (optional, if running migrations in container)
ENV PATH="${PATH}:/root/.dotnet/tools"

WORKDIR /app
COPY *.csproj ./
RUN dotnet restore

COPY . ./
RUN mkdir -p /var/dpkeysf
RUN dotnet publish -c Release -o out

# Run migrations if needed - it broke on docker compose build because it requires a database connection
# RUN dotnet ef database update

FROM --platform=$BUILDPLATFORM mcr.microsoft.com/dotnet/sdk:9.0 AS deploy

RUN dotnet tool install -g dotnet-ef

WORKDIR /app
COPY --from=build /app/out .
COPY --from=react-build /app/ClientApp/build ./ClientApp/build


EXPOSE 5000
ENTRYPOINT ["dotnet", "bloom.dll"]
