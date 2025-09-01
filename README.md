# ENME480 Course Site (MkDocs Material)

## Quickstart
```bash
pip install -r requirements.txt
mkdocs serve
```
Go to http://127.0.0.1:8000

## Deploying (GitHub Actions + mike)
- Push to `main` to publish the `fa2025` version as `latest` on the `gh-pages` branch.
- In **Settings → Pages**, set Source = **Deploy from a branch**, Branch = **gh-pages** (root).

## Submodule
Add the lab code as a submodule (once):
```bash
git submodule add https://github.com/ENME480/Lab-Code labs/Lab-Code
git commit -m "Add Lab-Code submodule"
```
If the submodule is private, create a PAT and set `GH_PAT` secret, then update the checkout step accordingly.
