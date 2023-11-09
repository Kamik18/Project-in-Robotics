const changelogTemplate = document.createElement("template");
changelogTemplate.innerHTML = `
<h4 class="mt-3">
    <span class="p-2"> Version 1.0.0</span> - 13/08/2023
</h4>
<ul class="mt-3 mb-5">
    <li class="ms-3">Creating the project structure</li>
    <li class="ms-3">Configure the pages</li>
    <p>...</p>
</ul>
<hr />

<h4 class="mt-3">
    <span class="p-2"> Version x.x.x</span> - xx/xx/xxxx
</h4>
<ul class="mt-3 mb-5">
    <li class="ms-3">task</li>
</ul>
<hr />
`;

class Changelog extends HTMLElement {
  constructor() {
    // Always call super first in constructor
    super();
  }

  connectedCallback() {
    const shadowRoot = this.attachShadow({ mode: "open" });
    shadowRoot.appendChild(changelogTemplate.content);
  }
}
customElements.define("changelog-component", Changelog);

// Set constants
$(".site-logo .bg-primary").text("V 1.0.0");
$(".docs-time").text("last update: 09-11-2023");
