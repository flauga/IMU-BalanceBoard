# Airtable data management (board registration + gameplay sessions)

The dashboards record data to an Airtable base ("Balance Board Analytics",
`appCoGPA1LTG7TO6g`) through a **Netlify Functions** proxy. The Airtable Personal
Access Token (PAT) lives only in a server-side env var — it never ships to the
browser, so it is safe even though the dashboards are public.

```
Browser (Netlify-hosted dashboards)
  └─ fetch('/.netlify/functions/...')        ← no token in the page
       └─ Netlify Function (holds AIRTABLE_PAT) → api.airtable.com
```

## Functions

| Endpoint | Method | Purpose |
|---|---|---|
| `/.netlify/functions/register-board`  | POST | Upsert a Balance Boards row by `MAC ID` (testing dashboard). |
| `/.netlify/functions/record-session`  | POST | Add a Gameplay Sessions row linked to the board (steadysteps, beginner levels 1–5). |
| `/.netlify/functions/leaderboard`     | GET  | Top sessions for a board, highest score first (steadysteps leaderboard). |

All three share `netlify/functions/_airtable.js`, which holds the table/field IDs
and the MAC→record-id resolution. Fields are addressed by ID so renaming a field
in Airtable does not break the calls.

## Setup

1. **Create a PAT** in Airtable scoped to this base with `data.records:read` +
   `data.records:write`.
2. **Local dev:** copy `.env.example` to `.env`, set `AIRTABLE_PAT`, then run
   `netlify dev` from the repo root and open `http://localhost:8888/testingdashboard.html`.
   (`.env` is gitignored.)
3. **Deploy:** connect the repo to Netlify. In Site settings → Environment
   variables set `AIRTABLE_PAT` and `AIRTABLE_BASE_ID`. Netlify publishes
   `dashboards/` at the site root and serves the functions automatically.

## What is recorded

- **Balance Boards:** `MAC ID`, `Device Model`, `Manufacture Date`, `Board Status`
  (one row per physical board; re-registering updates the same row).
- **Gameplay Sessions:** `Game Level` ("1".."5"), `Score`, and a link to the board.
  Only the 5 beginner levels are recorded today; arcade/daily plays are not.
