---
paths:
  - "**/*.{h,hpp,hh,c,cc,cpp,cxx}"
---

# C++ Style Requirements

These rules apply to all C and C++ source/header files.

## Formatting

All C/C++ code must follow the repository `.clang-format` file.

Current required style:

```yaml
BasedOnStyle: LLVM
IndentWidth: 4
TabWidth: 4
UseTab: Never
BreakBeforeBraces: Allman
AllowShortFunctionsOnASingleLine: None
```

When editing C/C++ files:

- Use 4-space indentation.
- Never use tabs.
- Use Allman braces.
- Do not place short functions on a single line.
- Prefer running `clang-format -i <file>` before finalizing edits.

## File header requirement

Every newly created C/C++ file must start with this Doxygen file comment:

```cpp
/**
 * @file <filename>
 * @brief <one-sentence description of this file>
 * @date <YYYY-MM-DD>
 */
```

Rules:

- Fill in `<filename>` with the actual file name.
- Fill in `<brief>` with a useful one-sentence description.
- Fill in `<YYYY-MM-DD>` with the current date.
- Do not leave empty Doxygen fields.
- When substantially modifying an existing C/C++ file that lacks this header, add it.

## Doxygen documentation requirement

Every function, struct, class, enum, and public type declaration must have a Doxygen comment.

For declarations in `.hpp` / `.h` files:

- Put the Doxygen comment in the header above the declaration.
- Do not duplicate the same full Doxygen comment in the `.cpp` definition unless implementation notes are needed.

For functions, structs, classes, enums, or private helpers defined only in a `.cpp` file:

- Put the Doxygen comment directly above the definition in the `.cpp` file.

Function comment style:

```cpp
/**
 * @brief Short description of what the function does.
 *
 * Longer explanation if needed.
 *
 * @param parameter_name Description of parameter.
 * @return Description of return value.
 */
```

Rules:

- Include `@param` for each parameter.
- Include `@return` for non-void functions.
- For pointers and references, document ownership, lifetime, and nullability when relevant.
- For embedded/firmware code, document blocking behavior, timing assumptions, thread safety, ISR safety, and units when relevant.

Struct/class comment style:

```cpp
/**
 * @brief Short description of what this type represents.
 *
 * Notes about ownership, invariants, units, or threading if relevant.
 */
struct Example
{
    /**
     * @brief Description of this field.
     */
    int field;
};
```

## Comment style

Do not write AI-looking separator comments.

Forbidden examples:

```cpp
// ------------------------------------------------------------
// ============================================================
// ************************************************************
// ===================== Helpers ==============================
// ------------ Section ------------
```

Do not use decorative ASCII section dividers made from repeated `-`, `=`, `*`, `_`, or `/`.

Use simple section comments instead:

```cpp
// Private helpers

// BLE packet construction

// Recorder sink setup
```

Doxygen block comments are allowed and preferred for declarations.

## Comment character style

Use plain ASCII notation in C/C++ comments and Doxygen comments.

Do not use Unicode superscript or subscript characters.

Forbidden examples:

```cpp
// acceleration in m/s²
// carrier density in cm⁻³
// x₀ is the initial position
// τₙ is the minority carrier lifetime
```

Use ASCII equivalents instead:

```cpp
// acceleration in m/s^2
// carrier density in cm^-3
// x_0 is the initial position
// tau_n is the minority carrier lifetime
```

Rules:

- Do not use Unicode superscript characters such as `²`, `³`, `⁻`, `⁺`, `⁰`, `¹`, `⁴`, `⁵`, `⁶`, `⁷`, `⁸`, `⁹`.
- Do not use Unicode subscript characters such as `₀`, `₁`, `₂`, `₃`, `ₙ`.
- Prefer ASCII math notation: `m/s^2`, `cm^-3`, `x_0`, `tau_n`, `Delta n`.
- Avoid Greek Unicode symbols in comments unless the surrounding codebase already uses them consistently.

## Final check before completing C/C++ work

Before finalizing C/C++ edits:

- Confirm new files have the required file header.
- Confirm declarations have Doxygen comments.
- Confirm `.cpp`-only symbols have Doxygen comments in the `.cpp` file.
- Confirm code follows `.clang-format`.
- Remove decorative separator comments.