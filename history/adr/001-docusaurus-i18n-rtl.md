# ADR 001: Docusaurus Internationalization and RTL Support Implementation

## Status
Accepted

## Context
The Homanoid Robotics Book Docusaurus site needed to support multiple languages, specifically English and Urdu, with proper right-to-left (RTL) text direction for Urdu. This required implementing internationalization (i18n) functionality that would allow users to switch between languages seamlessly, with all site content, navigation, and layout adapting appropriately.

## Decision
We decided to implement Docusaurus built-in i18n functionality with the following approach:

1. **Configuration**: Configure both English (en) and Urdu (ur) locales in `docusaurus.config.ts`
2. **RTL Support**: Set Urdu locale to use RTL direction with `localeConfigs`
3. **Translation Files**: Create JSON translation files for:
   - Navbar elements (`i18n/ur/docusaurus-theme-classic/navbar.json`)
   - Footer elements (`i18n/ur/docusaurus-theme-classic/footer.json`)
   - Homepage content (`i18n/ur/code.json`)
4. **Component Updates**: Use `<Translate>` components in React components for dynamic content
5. **CSS Styling**: Add RTL-specific CSS rules to handle text direction and layout

## Alternatives Considered

### Alternative 1: Custom Language Switcher
- Build a custom language switcher component from scratch
- Pros: Full control over behavior and UI
- Cons: More complex implementation, potential for bugs, not leveraging Docusaurus capabilities

### Alternative 2: Separate Site Builds
- Build separate sites for each language
- Pros: Simpler per-language customization
- Cons: Duplication of infrastructure, harder to maintain consistency, SEO challenges

### Alternative 3: Client-Side Translation
- Use client-side JavaScript libraries for translation
- Pros: Dynamic translation without page reloads
- Cons: SEO issues, slower initial load, potential for untranslatable content

## Consequences

### Positive
- Leverages Docusaurus' built-in i18n capabilities
- Proper SEO support with separate URLs per language
- Automatic RTL layout handling
- Consistent translation management
- Built-in language switcher component
- Support for both static and dynamic content translation

### Negative
- Requires build process for translation updates
- Additional complexity in maintaining translation files
- Potential for missing translations if new content is added without updating all locale files
- Larger bundle size due to multiple language support

## Implementation
- Docusaurus configuration updated with locale settings
- Translation files created in `i18n/ur/` directory
- Homepage component updated with `<Translate>` components
- Custom CSS added for RTL styling
- Language switcher automatically appears in navbar