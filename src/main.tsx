import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import App from './App';
import ProjectApp from './projects/ProjectApp';
import { startNativeUiSmokeFromProjectHome } from './robot/ui-smoke';

createRoot(document.getElementById('root')!).render(<StrictMode><ProjectApp renderWorkspace={props => <App {...props}/>}/></StrictMode>);
startNativeUiSmokeFromProjectHome();
