---
id: 001
title: "Docusaurus i18n Urdu Implementation"
stage: general
date: "2025-12-10"
surface: "agent"
model: "Claude Sonnet 4.5"
feature: "none"
branch: "master"
user: "user"
command: "/sp.implement"
labels: ["i18n", "urdu", "docusaurus", "localization", "rtl"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "Homanoid-Robotics-Book/src/pages/index.tsx"
  - "Homanoid-Robotics-Book/docusaurus.config.ts"
  - "Homanoid-Robotics-Book/src/css/custom.css"
  - "Homanoid-Robotics-Book/i18n/ur/docusaurus-theme-classic/navbar.json"
  - "Homanoid-Robotics-Book/i18n/ur/docusaurus-theme-classic/footer.json"
  - "Homanoid-Robotics-Book/i18n/ur/code.json"
tests: []
---

## PROMPT_TEXT

Update my Docusaurus homepage (index.tsx):

Set the homepage logo to robotics-logo.png from /img folder. Ensure <img> uses a valid string for TypeScript.

Make a functional language switch (English ↔ Urdu) so that:

Selecting Urdu switches all site text, navbar, homepage, and pages to Urdu (/i18n/ur/).

Text direction changes to RTL for Urdu.

Selecting English switches back to default English.

Use Docusaurus i18n and provide working code snippets for index.tsx and docusaurus.config.js.

## RESPONSE_TEXT

Successfully implemented Docusaurus i18n functionality for English ↔ Urdu language switching with RTL support:

1. Updated homepage logo to use `/img/robotics-logo.png` with proper TypeScript string in `index.tsx`
2. Configured Docusaurus i18n in `docusaurus.config.ts` with:
   - English and Urdu locales
   - Urdu set to RTL direction
   - Language dropdown in navbar
3. Added RTL CSS support in `custom.css` for proper text direction
4. Created Urdu translation files:
   - Navbar translations (`i18n/ur/docusaurus-theme-classic/navbar.json`)
   - Footer translations (`i18n/ur/docusaurus-theme-classic/footer.json`)
   - Homepage content translations (`i18n/ur/code.json`)
5. Updated homepage component with `<Translate>` components for proper internationalization

The implementation follows Docusaurus best practices for i18n and includes proper RTL styling for Urdu language support.

## OUTCOME

The Docusaurus site now has functional English ↔ Urdu language switching with:
- Proper RTL layout for Urdu
- Complete translation of homepage, navbar, and footer
- Language switcher dropdown in the navigation bar
- All configuration files properly set up for i18n