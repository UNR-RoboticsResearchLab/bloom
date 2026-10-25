Upload study recordings to Box
==============================

Uploads are manual. Recording or saving a session does not send it to Box.

Researcher workflow
-------------------

1. Stop recording and select **Save archive to Bloom**.
2. Select **Upload saved recording to Box**. This copies that saved revision;
   save again to include a newer recording.
3. Wait for the success message, then select **Open uploaded recording**.

For an older session, select **Load saved archives**, then **Upload to Box**.
Failures retain the Bloom archive for retry. A durable receipt prevents repeat
uploads of a completed revision. New archive revisions export separately.

Only audio is copied; scores, notes, and participant labels are not exported.
Filenames contain an opaque owner hash and study/revision UUIDs. Audio can still
contain identifying speech. Use the study's approved Box folder and access rules.
The integration does not create public shared links.

Administrator setup
-------------------

Provide the numeric folder ID from ``https://app.box.com/folder/<folder-id>``.
The destination is configured on the server, not selected by each researcher.

Create a Box Platform App using **Server Authentication with Client Credentials
Grant**, authorize it through the Box enterprise administrator, and give its
service account upload access to the folder. Follow the official
`Box setup instructions <https://developer.box.com/guides/authentication/client-credentials/client-credentials-setup/>`_.

Configure backend secrets through the deployment's secret store or private
``backend/.env`` (loaded by development Compose)::

   RsrStudy__Box__FolderId=<destination-folder-id>
   RsrStudy__Box__ClientId=<box-app-client-id>
   RsrStudy__Box__ClientSecret=<box-app-client-secret>
   RsrStudy__Box__SubjectId=<box-enterprise-id>
   RsrStudy__Box__SubjectType=enterprise

Keep the secret out of Git and React configuration. Recreate the backend
container after changing its environment. For an app approved to act as a
managed user, use ``SubjectType=user`` and the authorized user's ID instead.
Missing configuration produces a message when upload is clicked.

Recovery and limits
-------------------

Files up to 50 MB use Box's
`direct upload API <https://developer.box.com/reference/post-files-content/>`_;
larger files use chunked transfer within Bloom's existing archive limit.
The Box account's storage and file-size limits still apply.

Each revision stores a private ``box-upload.json`` receipt on the persistent
archive volume. Requests within one backend process are serialized. After a
lost response, an existing filename is reused only if its folder, name, size,
and SHA-1 match the archived audio. Different files are never overwritten.
Interrupted chunked uploads can leave temporary sessions until Box expires them.

Changing the destination does not move existing exports. If a Box file is later
deleted, its Bloom receipt remains; restoration or deliberate re-export requires
administrator intervention.

Live verification requires the actual folder ID and authorized Box app. Tests
use simulated HTTP responses and do not send study recordings to Box.
