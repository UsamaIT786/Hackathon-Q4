# Professional UI Components & Classes Reference

## 🎯 Quick Reference Guide

### Image Components

#### Image Card (with Overlay)
```html
<div class="image-card">
  <img src="image.jpg" alt="AI Generated Image">
  <div class="image-card-overlay">
    <p>Hover text description</p>
  </div>
</div>
```
**Effects:** Scale on hover, overlay fade-in, elevation shadow
**Perfect for:** AI-generated images, feature showcases

#### Featured Image
```html
<div class="featured-image">
  <img src="image.jpg" alt="Featured">
</div>
```
**Effects:** Slight rotation lift, 5% zoom, brightness boost
**Perfect for:** Hero images, primary visual elements

#### Floating Image
```html
<img src="image.jpg" alt="Floating" class="image-float">
```
**Effects:** Continuous up/down animation (4s cycle)
**Perfect for:** Decorative elements, drawing attention

### Button & Link Components

#### Gradient Button
```html
<button class="button-gradient">Click Me</button>
```
**Effects:** Ripple on hover, glow shadow, smooth elevation
**Perfect for:** Call-to-action, primary actions

#### Icon Button
```html
<button class="icon-button">🚀</button>
```
**Effects:** Scale + rotate (15°) on hover, glow effect
**Perfect for:** Quick actions, navigation

#### Underline Hover Link
```html
<a href="#" class="underline-hover">Learn More</a>
```
**Effects:** Gradient underline expands left-to-right
**Perfect for:** Navigation, inline links

### Card Components

#### Feature Card
```html
<div class="feature-card">
  <h3>Feature Title</h3>
  <p>Description text</p>
</div>
```
**Effects:** Shimmer sweep on hover, lift animation, border highlight
**Perfect for:** Highlighting features, callout boxes

### Text Effects

#### Gradient Text
```html
<span class="gradient-text">Important Text</span>
```
**Effect:** Blue to Green gradient, bold weight
**Perfect for:** Titles, key terms

#### Highlight Animated
```html
<span class="highlight-animated">Emphasized</span>
```
**Effect:** Expanding background highlight on hover
**Perfect for:** Keywords, important information

### Hero & Layout

#### Hero Section
```html
<div class="hero-section" style="background-image: url('ai-image.jpg');">
  <div class="hero-section-content">
    <h1>Welcome to Our Site</h1>
    <p>Professional introduction text</p>
  </div>
</div>
```
**Features:**
- Gradient overlay (opacity 85%)
- Parallax effect (fixed background)
- 500px minimum height
- Radial light gradient for depth

### Image Utilities

```html
<!-- Responsive image -->
<img src="image.jpg" class="img-responsive img-shadow img-rounded">

<!-- Bordered image -->
<img src="image.jpg" class="img-bordered">

<!-- Shadow only -->
<img src="image.jpg" class="img-shadow">
```

## 🎨 CSS Variables Reference

### Colors
```css
--ifm-color-primary: #0052CC (Blue)
--ifm-color-secondary: #10B981 (Green)
--ifm-color-info: #06B6D4 (Cyan)
--ifm-color-success: #10B981 (Green)
--ifm-color-warning: #F59E0B (Amber)
--ifm-color-danger: #EF4444 (Red)
```

### Timing Functions
```css
--transition-fast: 150ms cubic-bezier(0.4, 0, 0.2, 1)
--transition-smooth: 300ms cubic-bezier(0.4, 0, 0.2, 1)
--transition-bounce: 400ms cubic-bezier(0.34, 1.56, 0.64, 1)
```

### Shadows
```css
--shadow-xs: 0 0 1px rgba(0, 0, 0, 0.05)
--shadow-sm: 0 1px 2px 0 rgba(0, 0, 0, 0.05)
--shadow-md: Medium card shadow
--shadow-lg: Large button/hover shadow
--shadow-xl: Extra large elevation
--shadow-2xl: Maximum depth (hero, modal)
```

### Glow Effects
```css
--glow-primary: Blue glow (primary color)
--glow-secondary: Green glow (secondary color)
--glow-info: Cyan glow (info color)
```

## 💡 Best Practices

### Using Hover Effects
1. **Button Gradient** - Primary CTA buttons
2. **Icon Button** - Quick action shortcuts
3. **Feature Card** - Highlight important sections
4. **Image Card** - Showcase visual content

### Color Strategy
- Use **primary blue** for main interactions
- Use **secondary green** for success states
- Use **warning/danger** for alerts (sparingly)
- Maintain sufficient contrast ratio (WCAG AA)

### Animation Timing
- **Fast (150ms)** - Micro-interactions, hovers
- **Smooth (300ms)** - Page transitions, cards
- **Bounce (400ms)** - Celebratory, success states

### Responsive Design
- Mobile: Reduce scale animations (4% → hover scale instead of 8%)
- Mobile: Remove rotations
- Mobile: Maintain touch-friendly sizes (48px minimum)

### AI Image Integration
```html
<!-- Hero with AI image -->
<div class="hero-section" style="background-image: url('ai-generated/hero.jpg');">
  <div class="hero-section-content">
    <!-- Content -->
  </div>
</div>

<!-- Featured AI image -->
<div class="featured-image">
  <img src="ai-generated/robot.jpg" alt="Humanoid Robot">
</div>

<!-- Card with AI images -->
<div class="image-card">
  <img src="ai-generated/feature.jpg" alt="Feature">
  <div class="image-card-overlay">Learn More</div>
</div>
```

## 🌙 Dark Mode Support

All components automatically adapt to dark mode:
- Colors automatically switch to dark variants
- Shadows remain visible
- Text contrast maintained
- Smooth theme transition

## ✅ Accessibility Checklist

- [ ] Color contrast ratio ≥ 4.5:1 for text
- [ ] Focus states visible (keyboard navigation)
- [ ] Alt text on all images
- [ ] Semantic HTML structure
- [ ] Touch targets ≥ 48px (mobile)
- [ ] Animation respects `prefers-reduced-motion`

## 🚀 Performance Tips

1. **Use CSS variables** instead of hardcoding colors
2. **Leverage GPU acceleration** with transforms
3. **Minimize repaints** by grouping hover effects
4. **Defer heavy images** with lazy loading
5. **Use will-change** sparingly for animated elements

---

**Last Updated:** 2025-12-21
**Status:** Production Ready ✅
