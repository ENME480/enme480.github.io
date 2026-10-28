"""MkDocs build hooks for the ENME480 site.

Stamps every page with the build it came from and publishes that same id at
/version.txt, so docs/javascripts/refresh.js can tell an open tab that the page
it is showing has been superseded.

CI sets GITHUB_SHA, so the id matches the commit that produced the build. Local
builds fall back to a UTC timestamp.
"""

import datetime
import os
import pathlib

BUILD_ID = (
    os.environ.get("GITHUB_SHA", "")[:12]
    or datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d%H%M%S")
)


def on_post_page(output, page, config):
    """Put the build id in the head of every page."""
    tag = f'<meta name="site-build" content="{BUILD_ID}">'
    return output.replace("</head>", f"{tag}</head>", 1)


def on_post_build(config):
    """Publish the current build id for the page script to compare against."""
    path = pathlib.Path(config["site_dir"]) / "version.txt"
    path.write_text(BUILD_ID + "\n", encoding="utf-8")
