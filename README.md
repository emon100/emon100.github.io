# emon100.com

Personal website for Yimeng Wang (emon100). GitHub Pages publishes `master` from the repository root. `CNAME` retains `www.emon100.com`; the apex domain redirects there.

Roadster Lab is at `/lab/roadster/`. It is a static export of the Roadster project, including browser physics, models, source notes and recorded experiments. It is for personal learning and research, is unaffiliated with Tesla, and has not been validated on a real vehicle. JEV uses recorded runs on this static host; new API experiments require the original project's local service. Do not add API credentials to this repository.

To update Roadster, run `python3 site/scripts/sync-navigation.py` and `node site/test-publishing.mjs` in the Roadster project, then copy its `site/dist/` contents into `lab/roadster/`. Keep `.nojekyll` so Pages serves the export without processing it as Jekyll.

Preview from this repository with `python3 -m http.server 4174 --bind 127.0.0.1`. Check publishing with `node test-site.mjs`.
