# Architecture overview

[Interactive map](https://matemink.github.io/OnboardAutonomy/) ·
[Diagram source](overview.architecture.json)

The map describes the current observation prototype, with source links pinned
in `meta.repository.revision`. It is a snapshot of inspected code, not proof of
a complete simulator or hardware run.

Generated with [Archify v3.0.1](https://github.com/tt-a1i/archify/tree/v3.0.1)
(MIT; see [license](LICENSE.archify.txt)). Archify is an optional documentation
tool; the runtime and CI builds do not depend on it.

To refresh the map, inspect the affected runtime paths, update the JSON and its
source revision, then run from the repository root with Node.js 18+ and an
external checkout of Archify v3.0.1:

```bash
node /path/to/archify/skills/archify/bin/archify.mjs finalize architecture \
  docs/diagrams/overview.architecture.json docs/diagrams/overview.html \
  --repo-root . --quality showcase --json \
  --out-dir artifacts/archify-review
```

Require passing validation, artifact and browser checks. Open the HTML and
export **SVG · Light** and **SVG · Dark** as `overview-light.svg` and
`overview-dark.svg`. Inspect both exports and update them in the same commit
as the JSON and HTML. Both READMEs use these shared images.

The Pages workflow publishes the checked HTML and these assets from `main`;
it does not regenerate the map or publish runtime logs.
