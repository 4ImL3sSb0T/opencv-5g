# Repository Guidelines

## Project Structure & Modules
- `src/` contains all C++20 targets: `main.cpp` (cone detection entry), `guided_main.cpp`, `trace.cpp`, ONNX helpers, and shared headers such as `config.hpp`, `garage.hpp`, `cone_detector.hpp`, `image_Q.*`.  
- `src/car/` hosts detection modules (YOLO, arrow, A/B detectors) with small test drivers.  
- `config/config.json` holds runtime tuning values (vision, PID, servo/gimbal).  
- `docs/` provides ONNX usage and troubleshooting notes; `img/` stores sample videos/images for local testing; `model/` keeps ONNX weights; `script/` offers Python utilities for preprocessing and ONNX checks.  
- Build artifacts live in `build/` or `cmake-build-*`; `lib/onnxruntime-win-x64-1.23.2` supplies the bundled ONNX Runtime.

## Build, Test, and Development Commands
- Configure & build:  
  - `cmake -S . -B build`  
  - `cmake --build build`
- Run binaries from `build/` (Windows: append `.exe`):  
  - `opencv_5g` (main cone detection + GUI sliders)  
  - `guided`, `trace`, `line_detection`, `demo` (scenario-specific control loops)  
  - `onnx_test model/LR.onnx img/test.jpg --size 320x320` (DNN validation)  
  - `arrow_test`, `ab_test`, `test_yolo_infer` (module smoke tests)
- If OpenCV/spdlog are not found, set `OpenCV_DIR`/`spdlog_DIR`; ONNX uses `lib/onnxruntime-win-x64-1.23.2`.

## Coding Style & Naming Conventions
- Follow existing Allman braces and 4-space indentation; prefer `camelCase` for functions/vars and `PascalCase` for types.  
- Use `spdlog` for new logging instead of `std::cout` in runtime paths.  
- Keep configuration-driven behavior: read from `config.json` rather than hardcoding.  
- Favor `const` references, `auto` judiciously, and small helper functions inside related headers.

## Testing Guidelines
- No formal unit tests; validate via provided executables and sample media in `img/`.  
- For vision changes, run `opencv_5g` or `line_detection` on representative clips; for ONNX updates, run `onnx_test` with the target model and confirm input size (typically 320x320).  
- When adding detectors in `src/car/`, update or create matching test drivers and document expected outputs (e.g., bounding boxes, class scores).

## Commit & Pull Request Guidelines
- Use conventional commits with scopes observed in history (e.g., `feat(detector): add red cone tracker`, `refactor(car): split yolo helpers`).  
- PRs should summarize behavior change, mention affected executables, and link issues/tasks; include screenshots or short clips for vision results when possible.  
- Describe configuration changes (`config/config.json`) and any new model or asset paths that reviewers need to reproduce.

## Security & Configuration Tips
- Keep large models under `model/` and avoid committing private data; prefer `.gitignore` for temporary captures.  
- Ensure new code handles missing files gracefully (config, model, media) and avoids blocking operations in real-time loops.

## Serena Usage Prompts
- Start explorations with `get_symbols_overview` on specific files (use `depth=1` for top-level methods) and `find_symbol` when you know a name path.
- Use `search_for_pattern` when the symbol name/location is unknown; prefer scoped `relative_path` filters to reduce noise.
- Read only the needed symbol bodies (`include_body=True`) instead of whole files; rely on `find_referencing_symbols` to locate callers before edits.
- Prefer symbolic edits: `replace_symbol_body` to rewrite a function, `insert_before_symbol`/`insert_after_symbol` for imports or new definitions.
- Call `think_about_task_adherence` before edits and `think_about_whether_you_are_done` before wrapping up to keep changes focused.
- 尽可能使用 Serena 工具以提高效率；首次使用时需先初始化后再调用。
- 不熟悉的 API 先用 Context7 查询，避免凭空猜测。

## 尽可能使用中文回复我