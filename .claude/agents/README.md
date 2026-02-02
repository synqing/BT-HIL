# Agent Capability Matrix

**Version:** 1.0.0  
**Last Updated:** 2025-01-XX  
**Purpose:** Complete inventory of specialist agents with domain expertise and selection guidance

---

## Overview

This directory contains specialist agent personas for the Lightwave-Ledstrip project. Each agent has specific domain expertise and should be selected based on task requirements.

**MANDATORY PROTOCOL:** Before initiating any research, debugging, troubleshooting, or implementation task, you MUST:
1. Review this complete inventory
2. Analyze the domain expertise of each available agent
3. Strategically select the optimal combination of specialist agents for the task

---

## Agent Inventory

### Embedded Systems & Hardware

#### `embedded-system-engineer.md`
**Domain Expertise:**
- ESP32-S3 firmware architecture
- FreeRTOS threading and task management
- Mutex synchronization and thread safety
- FastLED dual-strip control (WS2812)
- Memory management (SRAM, Flash, NVS)
- Hardware peripherals (GPIO, I2C, RMT)
- Power management for 320 LEDs
- Performance profiling and optimization
- Feature flags and build configuration

**When to Select:**
- Hardware configuration changes
- FreeRTOS task creation/modification
- Memory allocation issues
- FastLED/RMT driver problems
- Performance optimization
- Feature flag modifications
- GPIO pin configuration
- Thread safety issues

**Specialist Routing:**
- Hardware config, pins, GPIO → **embedded-system-engineer**
- FreeRTOS tasks, mutex, threading → **embedded-system-engineer**
- FastLED, RMT, power management → **embedded-system-engineer**
- Feature flags, build config → **embedded-system-engineer**

---

### Visual Effects & Rendering

#### `visual-fx-architect.md`
**Domain Expertise:**
- FxEngine architecture and effect management
- Zone Composer system (multi-zone rendering)
- Transition Engine (12 transition types, 15 easing curves)
- 46 visual effects (LGP quantum/organic/geometric)
- Palette system (57 palettes with metadata)
- CENTER ORIGIN rendering patterns
- FastLED math functions and optimization
- Effect registration and lifecycle

**When to Select:**
- Creating new visual effects
- Modifying existing effects
- Zone Composer configuration
- Transition Engine changes
- Palette system modifications
- CENTER ORIGIN pattern implementation
- Effect performance optimization
- Visual rendering pipeline changes

**Specialist Routing:**
- Effect creation, EffectBase → **visual-fx-architect**
- Zone Composer, zone presets → **visual-fx-architect**
- Transition Engine, easing curves → **visual-fx-architect**
- Palettes, color science → **visual-fx-architect**
- LGP effects (quantum, organic) → **visual-fx-architect**

---

### Network & API

#### `network-api-engineer.md`
**Domain Expertise:**
- REST API v1 (46+ endpoints)
- WebSocket real-time control protocol
- WiFiManager and connection state machine
- ESP-NOW wireless protocol (9 wireless encoders)
- OpenAPI 3.0.3 specification
- Rate limiting and CORS configuration
- Network security and validation
- mDNS and captive portal

**When to Select:**
- REST API endpoint changes
- WebSocket protocol modifications
- WiFi connection issues
- ESP-NOW wireless encoder integration
- OpenAPI spec updates
- Rate limiting implementation
- CORS configuration
- Network security hardening

**Specialist Routing:**
- REST API endpoints → **network-api-engineer**
- WebSocket protocol → **network-api-engineer**
- WiFi connection, AP mode → **network-api-engineer**
- ESP-NOW wireless encoders → **network-api-engineer**
- OpenAPI specification → **network-api-engineer**
- Rate limiting, CORS → **network-api-engineer**

---

### Serial Interface & Telemetry

#### `serial-interface-engineer.md`
**Domain Expertise:**
- Serial command interface (115200 baud)
- Command parsing and routing
- Telemetry and diagnostics output
- Serial menu system
- Debug output formatting
- Performance metrics reporting

**When to Select:**
- Serial command additions/modifications
- Telemetry output changes
- Debug logging improvements
- Command parsing issues
- Serial menu enhancements

---

### Color & Palette Specialization

#### `palette-specialist.md`
**Domain Expertise:**
- Color science and palette design
- Palette metadata and categorization
- Color blending algorithms
- Palette transitions and cycling
- Color correction and enhancement
- LGP-specific color considerations

**When to Select:**
- New palette creation
- Palette system architecture
- Color blending improvements
- Color correction algorithms
- Palette metadata enhancements

---

### Web Development & UI

#### `agent-nextjs.md`
**Domain Expertise:**
- Next.js framework
- React components
- Server-side rendering
- API routes
- Web application architecture

**When to Select:**
- Next.js application development
- React component creation
- SSR implementation
- Web application architecture

#### `agent-lvgl-uiux.md`
**Domain Expertise:**
- LVGL embedded graphics library
- Touch interface design
- Embedded UI/UX patterns
- Display driver integration

**When to Select:**
- LVGL interface development
- Touch interface design
- Embedded display integration

---

### Backend & Infrastructure

#### `agent-convex.md`
**Domain Expertise:**
- Convex backend platform
- Database queries and mutations
- Real-time subscriptions
- Serverless functions

**When to Select:**
- Convex backend integration
- Database operations
- Real-time features
- Serverless function development

#### `agent-vercel.md`
**Domain Expertise:**
- Vercel deployment platform
- Edge functions
- Serverless architecture
- CI/CD pipelines

**When to Select:**
- Vercel deployment configuration
- Edge function development
- Serverless architecture
- Deployment automation

#### `agent-clerk.md`
**Domain Expertise:**
- Clerk authentication platform
- User management
- Session handling
- OAuth integration

**When to Select:**
- Authentication implementation
- User management features
- Session handling
- OAuth integration

---

## Agent Selection Decision Matrix

### Step 1: Identify Task Domain

| Task Type | Primary Agent | Secondary Agents |
|-----------|---------------|------------------|
| Hardware/GPIO changes | `embedded-system-engineer` | - |
| FreeRTOS/threading | `embedded-system-engineer` | - |
| Visual effect creation | `visual-fx-architect` | `palette-specialist` |
| Zone system changes | `visual-fx-architect` | - |
| REST API changes | `network-api-engineer` | - |
| WebSocket protocol | `network-api-engineer` | - |
| WiFi/network issues | `network-api-engineer` | `embedded-system-engineer` |
| Serial commands | `serial-interface-engineer` | - |
| Palette creation | `palette-specialist` | `visual-fx-architect` |
| Web dashboard | `agent-nextjs` | `network-api-engineer` |
| Embedded UI | `agent-lvgl-uiux` | `embedded-system-engineer` |

### Step 2: Analyze Task Complexity

**Simple Tasks (Single Agent):**
- Single domain change
- No cross-system dependencies
- Clear, focused scope

**Complex Tasks (Multi-Agent):**
- Multiple domains involved
- Cross-system integration
- Requires coordination

### Step 3: Determine Parallel Execution Potential

**Can tasks be parallelized?**
- ✅ Independent subsystems
- ✅ Different file sets
- ✅ No shared state
- ✅ Separate test files

**Must use sequential execution?**
- ❌ Tightly coupled changes
- ❌ Shared state dependencies
- ❌ Requires ordered execution
- ❌ Single file modifications

---

## Multi-Agent Coordination Patterns

### Pattern 1: Sequential Coordination
**Use when:** Tasks are dependent or must execute in order

```
Agent A (embedded-system-engineer) → Hardware changes
    ↓
Agent B (visual-fx-architect) → Effect updates using new hardware
    ↓
Agent C (network-api-engineer) → API exposure of new features
```

### Pattern 2: Parallel Coordination
**Use when:** Tasks are independent and can execute concurrently

```
Agent A (visual-fx-architect) → New effect implementation
Agent B (palette-specialist) → New palette creation
Agent C (network-api-engineer) → API endpoint updates
    ↓
Integration & Review
```

### Pattern 3: Hybrid Coordination
**Use when:** Some tasks are independent, others are dependent

```
Phase 1 (Parallel):
  Agent A (embedded-system-engineer) → Hardware optimization
  Agent B (visual-fx-architect) → Effect improvements
  
Phase 2 (Sequential):
  Agent C (network-api-engineer) → API integration
```

---

## Agent Capability Summary

| Agent | Primary Domain | Secondary Domains | Complexity Level |
|-------|---------------|-------------------|------------------|
| `embedded-system-engineer` | Hardware/Firmware | FreeRTOS, Memory, Performance | High |
| `visual-fx-architect` | Visual Effects | Zones, Transitions, Palettes | High |
| `network-api-engineer` | Network/API | WiFi, WebSocket, ESP-NOW | High |
| `serial-interface-engineer` | Serial Interface | Telemetry, Debugging | Medium |
| `palette-specialist` | Color Science | Palette Design, Blending | Medium |
| `agent-nextjs` | Web Development | React, SSR | Medium |
| `agent-lvgl-uiux` | Embedded UI | Touch, Display | Medium |
| `agent-convex` | Backend | Database, Real-time | Medium |
| `agent-vercel` | Deployment | Serverless, CI/CD | Low |
| `agent-clerk` | Authentication | User Management | Low |

---

## Version History

- **v1.0.0** (2025-01-XX): Initial capability matrix creation
  - Documented 10 specialist agents
  - Created selection decision matrix
  - Defined coordination patterns

---

## Related Documentation

- `CLAUDE.md` - Main agent guidance and protocol requirements
- `.claude/skills/dispatching-parallel-agents/SKILL.md` - Parallel execution protocol
- `.claude/skills/subagent-driven-development/SKILL.md` - Subagent workflow protocol
- `docs/AGENT_OPERATIONAL_PROTOCOL_AUDIT.md` - Complete protocol audit

---

**Remember:** Always review this inventory and strategically select agents before starting any task. Parallel execution should be used whenever tasks are independent and can benefit from concurrent processing.

