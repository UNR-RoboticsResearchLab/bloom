Deploy the study at dev-bloom.unr.dev
=====================================

The tunnel should forward ``https://dev-bloom.unr.dev`` to
``http://localhost:8081`` on the Docker host. The standalone study stack serves
the React pages, API, robot endpoints, and prompt audio through that one port.
No additional webserver route or browser-facing API port is needed.

On the LX host, Jenkins already occupies 8081. Set ``STUDY_PORT=8082`` in
``.env.study`` (the example file uses this value), and change the tunnel origin
to ``http://localhost:8082``. Substitute 8082 in the local checks below. Keep
Jenkins and the existing Bloom service running.

Prepare and start
-----------------

On the machine receiving tunnel traffic::

   cd /path/to/bloom
   git switch rsr-study-dashboard
   git pull --ff-only
   cp .env.study.example .env.study

Edit ``.env.study`` and replace both example database passwords with random
values before the first start. This file is ignored by Git. If automatic
analysis is available, set ``STUDY_ANALYSIS_URL`` to its address as reached from
inside the backend container. With no analysis service, recording, playback,
review, and archiving remain available; automatic analysis will fail until
configured. This stack does not start a GPU service.

Start the isolated deployment::

   docker compose --env-file .env.study -p bloom-study \
     -f docker-compose.study.yml up -d --build
   docker compose --env-file .env.study -p bloom-study \
     -f docker-compose.study.yml ps

Use the same project name and Compose file for future operations. The service
uses ``Dockerfile.study``; it embeds the study frontend in the backend's
``wwwroot`` and builds its API setting as ``same-origin``. Browser requests
therefore use ``https://dev-bloom.unr.dev/api/...``, not ``:8080`` or localhost.

The host binding is ``127.0.0.1:8081``. A tunnel process on the same host can
reach it. If the tunnel runs in a different container, its ``localhost`` refers
to that container; configure its origin networking appropriately before testing.

This Compose file supplies independent databases, authentication-key storage,
and audio-archive storage, all scoped to ``bloom-study``. It does not reuse or
replace the normal Bloom demo's containers, accounts, or volumes. Keep the
database passwords stable after initialization; changing an environment value
alone does not change a password stored in MariaDB.

Check routing and sign-in
-------------------------

On the Docker host::

   curl -I http://localhost:8081/rsr-study/login
   curl -f http://localhost:8081/api/rsr-speech/sentences

After starting the tunnel, check::

   curl -I https://dev-bloom.unr.dev/rsr-study/login
   curl -f https://dev-bloom.unr.dev/api/rsr-speech/sentences

Open ``https://dev-bloom.unr.dev/rsr-study/login``. Issue researcher accounts in
this deployment's database through an administrator at
``https://dev-bloom.unr.dev/rsr-study/accounts``. Accounts from local previews,
the old Docker stack, or ``bloom.unr.dev`` are not copied automatically.

Verify sign-in survives a page refresh and that the child page opens under the
same HTTPS origin. Rehearse recording, saving an archive, and playing it back
before using participant recordings. The tunnel must permit the archive upload
size and duration needed for a full study session; its upload limits may be
lower than Bloom's 200 MB request limit.

Point M at the same deployment
------------------------------

For this study server, set the existing entry in M's ``~/.bloom/robot.cfg`` to::

   base_url=https://dev-bloom.unr.dev

Stop the previous ROS launch, then launch one stack only::

   ros2 launch bloom_node m_sarg_combined.launch.py \
     base_url:=https://dev-bloom.unr.dev

The saved configuration and launch argument must match. Pair the current robot
code on the **dev-bloom** dashboard. A code created on ``bloom.unr.dev`` belongs
to a different database and cannot pair on this deployment. See
:doc:`m-direct-ethernet` for workspace sourcing and face troubleshooting.

Box and persistence
-------------------

The backend optionally loads the host's private ``backend/.env`` at container
creation. After Box approval, configure its backend variables as described in
:doc:`box-recordings`, then recreate ``study-web``. Box is not enabled merely
by deploying this stack.

Logs and shutdown::

   docker compose --env-file .env.study -p bloom-study \
     -f docker-compose.study.yml logs --tail=100 study-web
   docker compose --env-file .env.study -p bloom-study \
     -f docker-compose.study.yml down

Do not add ``-v``: that removes persistent databases, cookie keys, and recordings.
Back up the study volumes according to the study's storage procedure.

The stack currently uses Bloom's Development environment, matching its existing
container setup and persistent-key behavior. This is separate from the normal
demo; it is not a migration of that server's data.
