# Email confirmation

New accounts must confirm their email address before they can sign in
(`SignIn.RequireConfirmedAccount`, `User.RequireUniqueEmail`).

## Flow

1. `POST /api/user/create` registers the account and emails a link to
   `GET /api/user/confirm-email?userId=...&token=<Base64Url token>`.
2. The link confirms the account and shows a success or failure page. Tokens last 24 hours.
3. `POST /api/user/resend-confirmation` `{ "email": "..." }` sends a new link to an unconfirmed
   account. It always answers 200 with the same message (no account discovery) and is limited to
   3 requests per 10 minutes per client IP (429 after that).
4. Sign-in for an unconfirmed account returns 400 with `Code: "EmailNotConfirmed"`.

Students created by an SLP/Admin through `POST /api/user/student` also get a confirmation email and
cannot sign in until they confirm. Only the seeded `admin@example.com` / `slp@example.com` are created
already confirmed. The `ConfirmExistingEmails` migration confirms every account that exists before
the release.

## Forgot password

1. The React page `/forgot-password` calls `POST /api/user/forgot-password` `{ "email": "..." }`
   (3 per 10 minutes per IP, always the same 200 answer). A confirmed account is emailed a link; an
   unconfirmed one gets a confirmation link instead.
2. The link opens `GET /api/user/reset-password?userId=...&token=...`, a server-rendered form. Submitting
   it (`POST /api/user/reset-password`, form encoded) sets the password. Links last 24 hours and work once.

## Configuration

| Setting | Env var | Notes |
|---|---|---|
| `Smtp:Host` | `Smtp__Host` | Required to send. Empty = email not configured; registration still succeeds but the user is told the mail failed. |
| `Smtp:Port` | `Smtp__Port` | Default 587. The connection uses STARTTLS. |
| `Smtp:User` | `Smtp__User` | Leave empty if the relay needs no login. |
| `Smtp:Password` | `Smtp__Password` | Secret. |
| `Smtp:From` | `Smtp__From` | Sender, e.g. `Bloom <noreply@example.edu>`. |
| `App:PublicBaseUrl` | `App__PublicBaseUrl` | Public https origin used in links. If empty the request's scheme/host is used (corrected by `X-Forwarded-*`). |

With `docker-compose.prod.yml` set `SMTP_HOST`, `SMTP_PORT`, `SMTP_USER`, `SMTP_PASSWORD`, `SMTP_FROM`
and `PUBLIC_BASE_URL` in the server `.env` (see `.env.example`).

Local development uses user-secrets, so nothing goes in git:

```
cd backend
dotnet user-secrets set "Smtp:Host" "smtp.example.edu"
dotnet user-secrets set "Smtp:From" "Bloom <noreply@example.edu>"
dotnet user-secrets set "Smtp:User" "..."        # only if the relay requires login
dotnet user-secrets set "Smtp:Password" "..."
```

(User-secrets load only when running with `dotnet run` in the Development environment, not in Docker; there use env vars.)

## Proxy and keys

`X-Forwarded-For/Proto/Host` are honored, so the proxy must send them. Confirmation tokens are
protected by ASP.NET Data Protection, whose keys are persisted to `DataProtection:KeyringPath`
(`/var/dpkeys`, a volume), so links issued before a restart still work.
