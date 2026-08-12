# Generated architecture diagrams

These Mermaid diagrams are generated from the C++ AST by `clang-uml`; files
under `generated/` are local build artifacts and must not be edited manually.

Regenerate them after changing class or package relationships:

```bash
bash scripts/generate_diagrams.sh
```

The command creates `architecture_packages.mmd`, `autonomy_core.mmd`, and
`runtime_wiring.mmd`. Open them with a Mermaid preview extension or paste their
contents into any Mermaid-compatible viewer.
